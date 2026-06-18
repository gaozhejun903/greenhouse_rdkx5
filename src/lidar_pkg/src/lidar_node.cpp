/**
 * @file lidar_node.cpp
 * @brief ROS 2 LiDAR driver node with 0xAA55 protocol and outlier filtering
 *
 * Protocol (no CS field):
 *   PH(2) + CT(1) + LSN(1) + FSA(2) + LSA(2) + S{0..LSN-1}(3 each)
 *   Total packet size = 8 + LSN * 3
 * - PH: 0xAA 0x55
 * - CT[bit0]=1 marks start packet
 * - Sample bytes start at offset 8
 * - Distance(mm) = raw16 / 4
 * - Angle(deg)  = (raw16 >> 1) / 64
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include <string>
#include <vector>
#include <cmath>
#include <algorithm>
#include <cstring>
#include <cerrno>
#include <limits>
#include <chrono>
#include <thread>

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <asm/termbits.h>

#define LIDAR_BAUDRATE 150000
#define SCAN_INTERVAL_SEC 2.0   // watchdog: re-send start if no valid data
#define MAX_LSN 120             // sanity limit for packet samples

class LidarNode : public rclcpp::Node
{
public:
    LidarNode()
        : Node("lidar_node"), fd_(-1), is_shutdown_(false), scan_count_(0), last_point_angle_(0.0)
    {
        // 1. declare & get parameters
        this->declare_parameter<std::string>("port_name", "/dev/ttyACM0");
        this->declare_parameter<std::string>("frame_id", "laser");
        this->declare_parameter<bool>("filter.enabled", true);
        this->declare_parameter<double>("filter.radius", 0.10);
        this->declare_parameter<int>("filter.min_neighbors", 2);

        this->get_parameter("port_name", port_name_);
        this->get_parameter("frame_id", frame_id_);
        this->get_parameter("filter.enabled", filter_enabled_);
        this->get_parameter("filter.radius", filter_radius_);
        this->get_parameter("filter.min_neighbors", filter_min_neighbors_);

        // 2. publisher
        scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);
        full_scan_buffer_.reserve(1000);

        // 3. open serial
        if (!openSerial(port_name_, LIDAR_BAUDRATE)) {
            RCLCPP_FATAL(this->get_logger(), "Failed to open serial port: %s", port_name_.c_str());
            exit(1);
        }

        // 4. initialize LiDAR: stop -> wait -> flush -> start -> wait -> flush
        initLidar();

        RCLCPP_INFO(this->get_logger(), "Lidar node ready. Port: %s, Frame: %s", port_name_.c_str(), frame_id_.c_str());
        RCLCPP_INFO(this->get_logger(), "Filter: %s, Radius: %.2fm, MinNeighbors: %d",
            filter_enabled_ ? "ON" : "OFF", filter_radius_, filter_min_neighbors_);

        // 5. watchdog timer to re-send start command if no valid data
        watchdog_timer_ = this->create_wall_timer(
            std::chrono::duration<double>(SCAN_INTERVAL_SEC),
            std::bind(&LidarNode::watchdogCallback, this));
        last_valid_data_time_ = this->now();
    }

    ~LidarNode()
    {
        shutdown();
    }

    void run_loop()
    {
        uint8_t buffer[1024];
        rclcpp::Rate r(500);

        while (rclcpp::ok() && !is_shutdown_)
        {
            int n = read(fd_, buffer, sizeof(buffer));
            if (n > 0)
            {
                for (int i = 0; i < n; i++)
                    processByte(buffer[i]);
            }
            else if (n < 0)
            {
                if (errno != EAGAIN)
                    RCLCPP_WARN(this->get_logger(), "Serial read error: %s", strerror(errno));
            }

            rclcpp::spin_some(this->get_node_base_interface());
            r.sleep();
        }
    }

private:
    // --- ROS ---
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
    std::string port_name_;
    std::string frame_id_;

    // --- filter ---
    bool filter_enabled_;
    double filter_radius_;
    int filter_min_neighbors_;

    // --- serial ---
    int fd_;
    bool is_shutdown_;

    // --- scan ---
    int scan_count_ = 0;
    rclcpp::Time last_scan_pub_time_;

    // --- parser FSM ---
    enum State {
        WAIT_HEADER1,
        WAIT_HEADER2,
        READ_META,
        READ_PAYLOAD
    };
    State state_ = WAIT_HEADER1;
    std::vector<uint8_t> packet_buffer_;
    uint8_t current_lsn_ = 0;
    size_t target_payload_size_ = 0;

    // --- scan data ---
    struct LidarPoint {
        double angle_rad;
        double distance_m;
        double x, y; // cartesian for fast neighbor search
    };
    std::vector<LidarPoint> full_scan_buffer_;
    double last_point_angle_;
    rclcpp::Time scan_start_time_;
    bool first_packet_of_scan_ = true;

    // --- watchdog ---
    rclcpp::TimerBase::SharedPtr watchdog_timer_;
    rclcpp::Time last_valid_data_time_;

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
            close(fd_); return false;
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
            close(fd_); return false;
        }

        ioctl(fd_, TCFLSH, TCIOFLUSH);
        return true;
    }

    void sendCmd(const std::vector<uint8_t>& cmd)
    {
        if (fd_ != -1) {
            write(fd_, cmd.data(), cmd.size());
            ioctl(fd_, TCSBRK, 1);
        }
    }

    void flushSerial()
    {
        ioctl(fd_, TCFLSH, TCIOFLUSH);
    }

    void shutdown()
    {
        if (is_shutdown_) return;
        is_shutdown_ = true;

        if (fd_ != -1) {
            std::vector<uint8_t> stop_cmd = {0xA5, 0x00, 0xA5, 0x65, 0xA5, 0x65};
            write(fd_, stop_cmd.data(), stop_cmd.size());
            ioctl(fd_, TCSBRK, 1);
            close(fd_);
            fd_ = -1;
        }
    }

    void initLidar()
    {
        RCLCPP_INFO(this->get_logger(), "Initializing LiDAR...");

        std::vector<uint8_t> stop_cmd = {0xA5, 0x65};
        sendCmd(stop_cmd);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        flushSerial();

        std::vector<uint8_t> start_cmd = {0xA5, 0x60};
        sendCmd(start_cmd);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        flushSerial();

        RCLCPP_INFO(this->get_logger(), "LiDAR start command sent (A5 60).");
    }

    void watchdogCallback()
    {
        auto elapsed = (this->now() - last_valid_data_time_).seconds();
        if (elapsed > SCAN_INTERVAL_SEC)
        {
            RCLCPP_WARN(this->get_logger(), "No valid LiDAR data for %.1f sec. Re-sending start command...", elapsed);
            std::vector<uint8_t> stop_cmd = {0xA5, 0x65};
            sendCmd(stop_cmd);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            flushSerial();
            std::vector<uint8_t> start_cmd = {0xA5, 0x60};
            sendCmd(start_cmd);
            std::this_thread::sleep_for(std::chrono::milliseconds(300));
            flushSerial();
            last_valid_data_time_ = this->now();
        }
    }

    // ================================================
    // Protocol Parser
    // ================================================
    inline uint16_t bytesToUint16(const uint8_t* data)
    {
        return (static_cast<uint16_t>(data[1]) << 8) | data[0];
    }

    void processByte(uint8_t byte)
    {
        switch (state_)
        {
        case WAIT_HEADER1:
            if (byte == 0xAA) {
                state_ = WAIT_HEADER2;
                packet_buffer_.clear();
                packet_buffer_.push_back(byte);
            }
            break;

        case WAIT_HEADER2:
            if (byte == 0x55) {
                state_ = READ_META;
                packet_buffer_.push_back(byte);
            } else if (byte == 0xAA) {
                packet_buffer_.clear();
                packet_buffer_.push_back(byte);
            } else {
                state_ = WAIT_HEADER1;
                packet_buffer_.clear();
            }
            break;

        case READ_META:
            packet_buffer_.push_back(byte);
            if (packet_buffer_.size() == 4) {
                current_lsn_ = packet_buffer_[3];
                // Sanity check: LSN too large -> discard packet
                if (current_lsn_ == 0 || current_lsn_ > MAX_LSN) {
                    state_ = WAIT_HEADER1;
                    packet_buffer_.clear();
                    break;
                }
                target_payload_size_ = 4 + (current_lsn_ * 3); // FSA(2)+LSA(2)+samples
                state_ = READ_PAYLOAD;
            }
            break;

        case READ_PAYLOAD:
            packet_buffer_.push_back(byte);
            if (packet_buffer_.size() == 4 + target_payload_size_) {
                parsePacket(packet_buffer_);
                state_ = WAIT_HEADER1;
            }
            break;
        }
    }

    void parsePacket(const std::vector<uint8_t>& packet_data)
    {
        if (first_packet_of_scan_) {
            scan_start_time_ = this->now();
            first_packet_of_scan_ = false;
        }

        uint8_t lsn = packet_data[3];
        if (lsn == 0) return;

        uint16_t fsangle_raw = bytesToUint16(packet_data.data() + 4);
        uint16_t lsangle_raw = bytesToUint16(packet_data.data() + 6);

        double angle_start_deg = static_cast<double>(fsangle_raw >> 1) / 64.0;
        double angle_end_deg   = static_cast<double>(lsangle_raw >> 1) / 64.0;

        double diff_angle_deg = 0.0;
        if (lsn > 1) {
            diff_angle_deg = angle_end_deg - angle_start_deg;
            if (diff_angle_deg < 0) diff_angle_deg += 360.0;
        }

        for (int i = 0; i < lsn; ++i)
        {
            size_t offset = 8 + i * 3; // PH(2)+CT(1)+LSN(1)+FSA(2)+LSA(2) = 8 bytes header
            if (offset + 1 >= packet_data.size()) break;

            uint16_t dist_raw = bytesToUint16(packet_data.data() + offset);
            double distance_mm = static_cast<double>(dist_raw) / 4.0;
            double distance_m  = distance_mm / 1000.0;

            double angle_deg = angle_start_deg;
            if (lsn > 1) {
                angle_deg = (diff_angle_deg / (lsn - 1)) * i + angle_start_deg;
            }

            double angle_rad = angle_deg * M_PI / 180.0;

            // Filter out zero / too-close measurements (radar min range = 0.1m)
            if (distance_m > 0.02)
            {
                last_valid_data_time_ = this->now();

                // Detect revolution boundary (angle wraps 360 -> 0)
                if (angle_rad < last_point_angle_ - M_PI)
                {
                    if (scan_count_ > 0) {
                        publishScan();
                    } else {
                        RCLCPP_INFO(this->get_logger(), "Skipping first partial scan...");
                    }
                    scan_count_++;
                    full_scan_buffer_.clear();
                    first_packet_of_scan_ = true;
                    scan_start_time_ = this->now();
                }

                LidarPoint p;
                p.angle_rad   = angle_rad;
                p.distance_m  = distance_m;
                p.x = distance_m * std::cos(angle_rad);
                p.y = distance_m * std::sin(angle_rad);
                full_scan_buffer_.push_back(p);
                last_point_angle_ = angle_rad;
            }
        }
    }

    // ================================================
    // Outlier Filter
    // ================================================
    void removeOutliers(std::vector<LidarPoint>& points)
    {
        if (points.empty()) return;
        std::vector<LidarPoint> clean;
        clean.reserve(points.size());
        double r2 = filter_radius_ * filter_radius_;

        for (size_t i = 0; i < points.size(); ++i) {
            int neighbors = 0;
            for (size_t j = 0; j < points.size(); ++j) {
                double dx = points[i].x - points[j].x;
                double dy = points[i].y - points[j].y;
                if (dx*dx + dy*dy < r2) neighbors++;
                if (neighbors >= filter_min_neighbors_) break;
            }
            if (neighbors >= filter_min_neighbors_)
                clean.push_back(points[i]);
        }
        points = std::move(clean);
    }

    // ================================================
    // Publish
    // ================================================
    void publishScan()
    {
        if (full_scan_buffer_.empty()) return;

        // Apply filter
        if (filter_enabled_) {
            size_t raw = full_scan_buffer_.size();
            removeOutliers(full_scan_buffer_);
            RCLCPP_DEBUG(this->get_logger(), "Filtered: %zu -> %zu points", raw, full_scan_buffer_.size());
        }

        sensor_msgs::msg::LaserScan scan;
        scan.header.stamp = (scan_start_time_.nanoseconds() == 0) ? this->now() : scan_start_time_;
        scan.header.frame_id = frame_id_;

        scan.angle_min = 0.0;
        scan.angle_max = 2.0 * M_PI;
        const int SCAN_SIZE = 720;
        scan.angle_increment = (2.0 * M_PI) / SCAN_SIZE;
        scan.range_min = 0.05;
        scan.range_max = 8.0;

        scan.ranges.assign(SCAN_SIZE, std::numeric_limits<float>::infinity());

        for (const auto& pt : full_scan_buffer_)
        {
            double idx_angle = 2.0 * M_PI - pt.angle_rad;
            if (idx_angle >= 2.0 * M_PI) idx_angle -= 2.0 * M_PI;
            if (idx_angle < 0) idx_angle += 2.0 * M_PI;

            int idx = static_cast<int>(idx_angle / scan.angle_increment);
            if (idx >= 0 && idx < SCAN_SIZE)
            {
                if (std::isinf(scan.ranges[idx]) || pt.distance_m < scan.ranges[idx])
                    scan.ranges[idx] = static_cast<float>(pt.distance_m);
            }
        }

        // Dynamically calculate scan_time from actual duration
        if (last_scan_pub_time_.nanoseconds() == 0) {
            scan.scan_time = 1.0 / 7.0; // default fallback
        } else {
            double dt = (this->now() - last_scan_pub_time_).seconds();
            scan.scan_time = std::max(dt, 0.01);
        }
        scan.time_increment = scan.scan_time / SCAN_SIZE;
        last_scan_pub_time_ = this->now();

        scan_pub_->publish(scan);

        // Log some stats
        int valid = 0;
        for (auto& r : scan.ranges) if (!std::isinf(r)) valid++;
        RCLCPP_INFO(this->get_logger(), "Published scan #%d: %d valid points out of %d",
            scan_count_, valid, SCAN_SIZE);
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LidarNode>();
    try {
        node->run_loop();
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Exception: %s", e.what());
    }
    rclcpp::shutdown();
    return 0;
}
