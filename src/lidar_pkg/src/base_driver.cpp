/**
 * @file base_driver.cpp
 * @brief STM32 底盘驱动节点
 *
 * 通过 USB 串口与 STM32 通信，控制小车运动。
 * 通信协议：AA + CMD + 55
 *   AA 00 55 → 前进
 *   AA 01 55 → 停止
 *   AA 02 55 → 后退
 *   AA 03 55 → 左转 (逆时针)
 *   AA 04 55 → 右转 (顺时针)
 *
 * 订阅 /cmd_vel (geometry_msgs/Twist) 接收导航速度指令
 * 发布 /odom (nav_msgs/Odometry) + TF odom→base_link (航位推算)
 */

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <string>
#include <vector>
#include <cmath>
#include <cerrno>
#include <cstring>
#include <chrono>
#include <memory>

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <asm/termbits.h>

class BaseDriver : public rclcpp::Node
{
public:
    BaseDriver()
        : Node("base_driver"),
          fd_(-1),
          x_(0.0), y_(0.0), theta_(0.0),
          last_cmd_linear_x_(0.0), last_cmd_angular_z_(0.0)
    {
        // 1. declare parameters
        this->declare_parameter<std::string>("port", "/dev/ttyUSB0");
        this->declare_parameter<int>("baudrate", 115200);
        this->declare_parameter<std::string>("odom_frame", "odom");
        this->declare_parameter<std::string>("base_frame", "base_link");
        this->declare_parameter<double>("wheel_base", 0.2);
        this->declare_parameter<double>("max_linear_speed", 0.5);
        this->declare_parameter<double>("max_angular_speed", 1.0);
        this->declare_parameter<double>("cmd_timeout", 0.5);

        this->get_parameter("port", port_);
        this->get_parameter("baudrate", baudrate_);
        this->get_parameter("odom_frame", odom_frame_);
        this->get_parameter("base_frame", base_frame_);
        this->get_parameter("max_linear_speed", max_linear_speed_);
        this->get_parameter("max_angular_speed", max_angular_speed_);
        this->get_parameter("cmd_timeout", cmd_timeout_);

        // 2. open serial
        if (!openSerial(port_, baudrate_)) {
            RCLCPP_FATAL(this->get_logger(), "Failed to open serial port: %s", port_.c_str());
            exit(1);
        }
        RCLCPP_INFO(this->get_logger(), "Base driver ready. Port: %s, Baud: %d", port_.c_str(), baudrate_);

        // 3. subscribe to raw byte commands (direct STM32 control)
        cmd_byte_sub_ = this->create_subscription<std_msgs::msg::UInt8>(
            "cmd_byte", 10,
            std::bind(&BaseDriver::cmdByteCallback, this, std::placeholders::_1));

    // 4. subscribe to /cmd_vel (for Nav2 navigation)
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10,
            std::bind(&BaseDriver::cmdVelCallback, this, std::placeholders::_1));

        // 4. odom publisher + TF broadcaster
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // 5. timer: publish odometry at 20Hz
        odom_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&BaseDriver::odomTimerCallback, this));

        // 6. timer: check cmd timeout at 10Hz
        timeout_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&BaseDriver::timeoutCheckCallback, this));

        last_cmd_time_ = this->now();
        last_odom_time_ = this->now();
    }

    ~BaseDriver()
    {
        if (fd_ != -1) {
            sendCmd(0x01); // stop before exit
            close(fd_);
        }
    }

private:
    // --- serial ---
    int fd_;
    std::string port_;
    int baudrate_;

    // --- ROS ---
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr cmd_byte_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr odom_timer_;
    rclcpp::TimerBase::SharedPtr timeout_timer_;

    std::string odom_frame_;
    std::string base_frame_;

    // --- odometry dead reckoning ---
    double x_, y_, theta_;
    rclcpp::Time last_cmd_time_;
    rclcpp::Time last_odom_time_;
    double last_cmd_linear_x_;
    double last_cmd_angular_z_;

    // --- config ---
    double max_linear_speed_;
    double max_angular_speed_;
    double cmd_timeout_;

    // ================================================
    // Serial
    // ================================================
    bool openSerial(const std::string& port, int baudrate)
    {
        fd_ = open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
        if (fd_ == -1) {
            RCLCPP_ERROR(this->get_logger(), "Open %s: %s", port.c_str(), strerror(errno));
            return false;
        }

        struct termios2 tio;
        if (ioctl(fd_, TCGETS2, &tio) != 0) {
            close(fd_);
            return false;
        }

        tio.c_cflag &= ~CBAUD;
        tio.c_cflag |= BOTHER;
        tio.c_ispeed = baudrate;
        tio.c_ospeed = baudrate;
        tio.c_cflag &= ~PARENB;
        tio.c_cflag &= ~CSTOPB;
        tio.c_cflag &= ~CSIZE;
        tio.c_cflag |= CS8;
        tio.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        tio.c_iflag &= ~(IXON | IXOFF | IXANY);
        tio.c_oflag &= ~OPOST;

        if (ioctl(fd_, TCSETS2, &tio) != 0) {
            close(fd_);
            return false;
        }

        ioctl(fd_, TCFLSH, TCIOFLUSH);
        return true;
    }

    void sendCmd(uint8_t cmd_byte)
    {
        if (fd_ == -1) return;
        uint8_t buf[3] = {0xAA, cmd_byte, 0x55};
        write(fd_, buf, 3);
        ioctl(fd_, TCSBRK, 1);
    }

    // ================================================
    // cmd_byte → STM32 command (direct byte control)
    // ================================================
    void cmdByteCallback(const std_msgs::msg::UInt8::SharedPtr msg)
    {
        sendCmd(msg->data);
        RCLCPP_INFO(this->get_logger(), "cmd_byte: 0x%02X", msg->data);
    }

    // ================================================
    // cmd_vel → STM32 command
    // ================================================
    uint8_t twistToCmd(double linear_x, double angular_z)
    {
        // Forward / backward takes priority
        if (linear_x > 0.05) {
            return 0x00; // forward
        } else if (linear_x < -0.05) {
            return 0x02; // backward
        } else if (angular_z > 0.05) {
            return 0x03; // rotate left (CCW)
        } else if (angular_z < -0.05) {
            return 0x04; // rotate right (CW)
        } else {
            return 0x01; // stop
        }
    }

    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        // Clamp to max speeds
        double vx = std::clamp(msg->linear.x, -max_linear_speed_, max_linear_speed_);
        double wz = std::clamp(msg->angular.z, -max_angular_speed_, max_angular_speed_);

        uint8_t cmd = twistToCmd(vx, wz);
        sendCmd(cmd);

        last_cmd_linear_x_ = vx;
        last_cmd_angular_z_ = wz;
        last_cmd_time_ = this->now();

        RCLCPP_DEBUG(this->get_logger(), "cmd_vel: v=%.2f, w=%.2f -> cmd=0x%02X", vx, wz, cmd);
    }

    void timeoutCheckCallback()
    {
        auto elapsed = (this->now() - last_cmd_time_).seconds();
        if (elapsed > cmd_timeout_)
        {
            // No cmd_vel received recently → stop the robot
            if (last_cmd_linear_x_ != 0.0 || last_cmd_angular_z_ != 0.0) {
                sendCmd(0x01); // stop
                last_cmd_linear_x_ = 0.0;
                last_cmd_angular_z_ = 0.0;
                RCLCPP_DEBUG(this->get_logger(), "cmd timeout: stopping robot");
            }
        }
    }

    // ================================================
    // Odom (dead reckoning from commanded velocity)
    // ================================================
    void odomTimerCallback()
    {
        auto now = this->now();
        double dt = (now - last_odom_time_).seconds();
        last_odom_time_ = now;

        // Integrate pose from last commanded velocity
        double vx = last_cmd_linear_x_;
        double wz = last_cmd_angular_z_;

        if (std::abs(vx) > 0.001 || std::abs(wz) > 0.001) {
            double dx = vx * std::cos(theta_) * dt;
            double dy = vx * std::sin(theta_) * dt;
            double dtheta = wz * dt;

            x_ += dx;
            y_ += dy;
            theta_ += dtheta;

            // Normalize theta
            if (theta_ > M_PI) theta_ -= 2.0 * M_PI;
            if (theta_ < -M_PI) theta_ += 2.0 * M_PI;
        }

        // Publish odometry message
        nav_msgs::msg::Odometry odom;
        odom.header.stamp = now;
        odom.header.frame_id = odom_frame_;
        odom.child_frame_id = base_frame_;

        odom.pose.pose.position.x = x_;
        odom.pose.pose.position.y = y_;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = quaternionFromYaw(theta_);

        // Velocity: reported as the commanded value
        odom.twist.twist.linear.x = vx;
        odom.twist.twist.angular.z = wz;

        // Set covariance (large values = high uncertainty, SLAM will correct)
        for (int i = 0; i < 36; i++) {
            odom.pose.covariance[i] = 0.0;
            odom.twist.covariance[i] = 0.0;
        }
        odom.pose.covariance[0] = 0.1;   // x
        odom.pose.covariance[7] = 0.1;   // y
        odom.pose.covariance[35] = 0.1;  // theta

        odom_pub_->publish(odom);

        // Publish TF: odom → base_link
        geometry_msgs::msg::TransformStamped tf;
        tf.header.stamp = now;
        tf.header.frame_id = odom_frame_;
        tf.child_frame_id = base_frame_;
        tf.transform.translation.x = x_;
        tf.transform.translation.y = y_;
        tf.transform.translation.z = 0.0;
        tf.transform.rotation = odom.pose.pose.orientation;

        tf_broadcaster_->sendTransform(tf);
    }

    // ================================================
    // Helper
    // ================================================
    geometry_msgs::msg::Quaternion quaternionFromYaw(double yaw)
    {
        geometry_msgs::msg::Quaternion q;
        q.x = 0.0;
        q.y = 0.0;
        q.z = std::sin(yaw / 2.0);
        q.w = std::cos(yaw / 2.0);
        return q;
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BaseDriver>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
