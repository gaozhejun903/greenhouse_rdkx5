/**
 * @file pest_monitor.cpp
 * @brief 病虫害定位监控节点
 *
 * 功能：
 *   1. 订阅 /amcl_pose 实时跟踪小车在地图中的位置
 *   2. 订阅 /pest_detected (视觉模型发布) 接收病虫害检测事件
 *   3. 检测到病虫害时，结合当前 AMCL 位姿记录病虫害地图坐标
 *   4. 发布 /pest_report (PestReport) 供其他节点使用
 *   5. 发布 /pest_markers (MarkerArray) 在 RViz 中可视化病虫害位置
 *   6. 通过 TCP 局域网上传病虫害数据到电脑端 Python 接收程序
 *
 * 话题：
 *   订阅 /amcl_pose        (geometry_msgs/PoseWithCovarianceStamped)
 *   订阅 /pest_detected     (lidar_pkg/PestDetection)
 *   发布 /pest_report       (lidar_pkg/PestReport)
 *   发布 /pest_markers      (visualization_msgs/MarkerArray)
 *   发布 /robot_pose        (geometry_msgs/PoseStamped) — 小车当前位置(便于调试)
 *
 * 参数：
 *   server_ip   — 电脑端 TCP 服务器 IP (默认 "192.168.1.100")
 *   server_port — 电脑端 TCP 服务器端口 (默认 8888)
 *   tcp_enabled — 是否启用 TCP 上传 (默认 true)
 *   log_pose_interval — 位姿日志打印间隔秒数 (默认 5.0)
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/string.hpp>

#include "lidar_pkg/msg/pest_detection.hpp"
#include "lidar_pkg/msg/pest_report.hpp"

#include <string>
#include <vector>
#include <cmath>
#include <cstring>
#include <chrono>
#include <sstream>
#include <iomanip>

#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>

using PestDetection = lidar_pkg::msg::PestDetection;
using PestReport = lidar_pkg::msg::PestReport;

class PestMonitor : public rclcpp::Node
{
public:
    PestMonitor()
        : Node("pest_monitor"),
          tcp_socket_(-1),
          current_x_(0.0), current_y_(0.0), current_theta_(0.0),
          pose_received_(false),
          marker_id_(0)
    {
        // 声明参数
        this->declare_parameter<std::string>("server_ip", "192.168.1.100");
        this->declare_parameter<int>("server_port", 8888);
        this->declare_parameter<bool>("tcp_enabled", true);
        this->declare_parameter<double>("log_pose_interval", 5.0);

        this->get_parameter("server_ip", server_ip_);
        this->get_parameter("server_port", server_port_);
        this->get_parameter("tcp_enabled", tcp_enabled_);
        this->get_parameter("log_pose_interval", log_pose_interval_);

        // 订阅 AMCL 位姿
        amcl_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "amcl_pose", 10,
            std::bind(&PestMonitor::amclPoseCallback, this, std::placeholders::_1));

        // 订阅病虫害检测消息
        pest_detected_sub_ = this->create_subscription<PestDetection>(
            "pest_detected", 10,
            std::bind(&PestMonitor::pestDetectedCallback, this, std::placeholders::_1));

        // 发布病虫害报告
        pest_report_pub_ = this->create_publisher<PestReport>("pest_report", 10);

        // 发布 RViz 标记
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("pest_markers", 10);

        // 发布小车当前位姿（便于调试）
        robot_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("robot_pose", 10);

        // TCP 重连定时器 (5 秒)
        if (tcp_enabled_) {
            tcp_timer_ = this->create_wall_timer(
                std::chrono::seconds(5),
                std::bind(&PestMonitor::tcpReconnectCallback, this));
            connectTcp();
        }

        // 位姿日志定时器
        pose_log_timer_ = this->create_wall_timer(
            std::chrono::duration<double>(log_pose_interval_),
            std::bind(&PestMonitor::poseLogCallback, this));

        RCLCPP_INFO(this->get_logger(), "PestMonitor 启动完成");
        RCLCPP_INFO(this->get_logger(), "TCP 上传: %s, 服务器: %s:%d",
            tcp_enabled_ ? "启用" : "禁用", server_ip_.c_str(), server_port_);
    }

    ~PestMonitor()
    {
        if (tcp_socket_ != -1) {
            close(tcp_socket_);
        }
    }

private:
    // --- ROS 接口 ---
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_pose_sub_;
    rclcpp::Subscription<PestDetection>::SharedPtr pest_detected_sub_;
    rclcpp::Publisher<PestReport>::SharedPtr pest_report_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr robot_pose_pub_;
    rclcpp::TimerBase::SharedPtr tcp_timer_;
    rclcpp::TimerBase::SharedPtr pose_log_timer_;

    // --- TCP ---
    int tcp_socket_;
    std::string server_ip_;
    int server_port_;
    bool tcp_enabled_;

    // --- 位姿 ---
    double current_x_, current_y_, current_theta_;
    bool pose_received_;
    double log_pose_interval_;

    // --- 病虫害记录 ---
    struct PestRecord {
        uint8_t pest_type;
        std::string pest_type_name;
        float confidence;
        double map_x, map_y, map_theta;
        std::string timestamp;
        std::string image_path;
    };
    std::vector<PestRecord> pest_records_;
    int marker_id_;

    // ================================================
    // AMCL 位姿回调
    // ================================================
    void amclPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        current_x_ = msg->pose.pose.position.x;
        current_y_ = msg->pose.pose.position.y;
        current_theta_ = quaternionToYaw(msg->pose.pose.orientation);
        pose_received_ = true;

        // 发布 PoseStamped 便于调试
        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header = msg->header;
        pose_msg.header.frame_id = "map";
        pose_msg.pose = msg->pose.pose;
        robot_pose_pub_->publish(pose_msg);
    }

    // ================================================
    // 病虫害检测回调 — 核心逻辑
    // ================================================
    void pestDetectedCallback(const PestDetection::SharedPtr msg)
    {
        if (!pose_received_) {
            RCLCPP_WARN(this->get_logger(), "收到病虫害检测消息，但尚未收到 AMCL 位姿，无法记录坐标");
            return;
        }

        // 构建病虫害记录
        PestRecord record;
        record.pest_type = msg->pest_type;
        record.pest_type_name = msg->pest_type_name;
        record.confidence = msg->confidence;
        record.map_x = current_x_;
        record.map_y = current_y_;
        record.map_theta = current_theta_;
        record.timestamp = msg->timestamp.empty() ? getCurrentTimeStr() : msg->timestamp;
        record.image_path = msg->image_path;

        pest_records_.push_back(record);

        RCLCPP_INFO(this->get_logger(),
            "病虫害检测 #%zu: 类型=%d (%s), 置信度=%.2f, 坐标=(%.2f, %.2f, %.2f)",
            pest_records_.size(), record.pest_type, record.pest_type_name.c_str(),
            record.confidence, record.map_x, record.map_y, record.map_theta);

        // 发布 PestReport 消息
        PestReport report;
        report.pest_type = record.pest_type;
        report.pest_type_name = record.pest_type_name;
        report.confidence = record.confidence;
        report.map_x = record.map_x;
        report.map_y = record.map_y;
        report.map_theta = record.map_theta;
        report.timestamp = record.timestamp;
        report.image_path = record.image_path;
        pest_report_pub_->publish(report);

        // 发布 RViz 标记
        publishMarker(record);

        // TCP 上传
        if (tcp_enabled_ && tcp_socket_ != -1) {
            std::string json = buildJsonReport(record);
            sendTcpData(json);
        } else if (tcp_enabled_) {
            RCLCPP_WARN(this->get_logger(), "TCP 未连接，数据仅本地记录");
        }
    }

    // ================================================
    // RViz 标记发布
    // ================================================
    void publishMarker(const PestRecord& record)
    {
        visualization_msgs::msg::MarkerArray marker_array;

        // 球体标记 — 标记病虫害位置
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = this->now();
        marker.ns = "pest_locations";
        marker.id = marker_id_++;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.pose.position.x = record.map_x;
        marker.pose.position.y = record.map_y;
        marker.pose.position.z = 0.1;
        marker.pose.orientation.w = 1.0;

        marker.scale.x = 0.2;
        marker.scale.y = 0.2;
        marker.scale.z = 0.2;

        // 根据病虫害类型分配颜色
        uint32_t type = record.pest_type % 6;
        switch (type) {
            case 0: marker.color.r = 1.0f; marker.color.g = 0.0f; marker.color.b = 0.0f; break; // 红
            case 1: marker.color.r = 0.0f; marker.color.g = 1.0f; marker.color.b = 0.0f; break; // 绿
            case 2: marker.color.r = 0.0f; marker.color.g = 0.0f; marker.color.b = 1.0f; break; // 蓝
            case 3: marker.color.r = 1.0f; marker.color.g = 1.0f; marker.color.b = 0.0f; break; // 黄
            case 4: marker.color.r = 1.0f; marker.color.g = 0.0f; marker.color.b = 1.0f; break; // 紫
            case 5: marker.color.r = 0.0f; marker.color.g = 1.0f; marker.color.b = 1.0f; break; // 青
        }
        marker.color.a = 1.0f;
        marker.lifetime = rclcpp::Duration(0, 0); // 永久

        marker_array.markers.push_back(marker);

        // 文字标记 — 显示类型编号
        visualization_msgs::msg::Marker text_marker;
        text_marker.header.frame_id = "map";
        text_marker.header.stamp = this->now();
        text_marker.ns = "pest_labels";
        text_marker.id = marker_id_++;
        text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        text_marker.action = visualization_msgs::msg::Marker::ADD;

        text_marker.pose.position.x = record.map_x;
        text_marker.pose.position.y = record.map_y;
        text_marker.pose.position.z = 0.35;
        text_marker.pose.orientation.w = 1.0;

        text_marker.scale.z = 0.15;

        std::ostringstream oss;
        oss << "#" << pest_records_.size() << " type=" << static_cast<int>(record.pest_type);
        if (!record.pest_type_name.empty()) {
            oss << " (" << record.pest_type_name << ")";
        }
        text_marker.text = oss.str();

        text_marker.color.r = 1.0f;
        text_marker.color.g = 1.0f;
        text_marker.color.b = 1.0f;
        text_marker.color.a = 1.0f;
        text_marker.lifetime = rclcpp::Duration(0, 0);

        marker_array.markers.push_back(text_marker);

        marker_pub_->publish(marker_array);
    }

    // ================================================
    // TCP 局域网上传
    // ================================================
    void connectTcp()
    {
        tcp_socket_ = socket(AF_INET, SOCK_STREAM, 0);
        if (tcp_socket_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "TCP socket 创建失败: %s", strerror(errno));
            return;
        }

        // 设置连接超时 (3 秒)
        struct timeval tv;
        tv.tv_sec = 3;
        tv.tv_usec = 0;
        setsockopt(tcp_socket_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
        setsockopt(tcp_socket_, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));

        struct sockaddr_in server_addr;
        memset(&server_addr, 0, sizeof(server_addr));
        server_addr.sin_family = AF_INET;
        server_addr.sin_port = htons(server_port_);

        if (inet_pton(AF_INET, server_ip_.c_str(), &server_addr.sin_addr) <= 0) {
            RCLCPP_ERROR(this->get_logger(), "无效的 IP 地址: %s", server_ip_.c_str());
            close(tcp_socket_);
            tcp_socket_ = -1;
            return;
        }

        if (connect(tcp_socket_, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
            RCLCPP_WARN(this->get_logger(), "TCP 连接失败: %s:%d (%s)",
                server_ip_.c_str(), server_port_, strerror(errno));
            close(tcp_socket_);
            tcp_socket_ = -1;
            return;
        }

        RCLCPP_INFO(this->get_logger(), "TCP 已连接: %s:%d", server_ip_.c_str(), server_port_);
    }

    void tcpReconnectCallback()
    {
        if (tcp_socket_ == -1) {
            RCLCPP_INFO(this->get_logger(), "尝试重新连接 TCP 服务器...");
            connectTcp();
        }
    }

    void sendTcpData(const std::string& data)
    {
        if (tcp_socket_ == -1) return;

        std::string msg = data + "\n";
        ssize_t sent = send(tcp_socket_, msg.c_str(), msg.size(), MSG_NOSIGNAL);
        if (sent < 0) {
            RCLCPP_WARN(this->get_logger(), "TCP 发送失败: %s", strerror(errno));
            close(tcp_socket_);
            tcp_socket_ = -1;
        } else {
            RCLCPP_INFO(this->get_logger(), "TCP 数据已上传 (%zd bytes)", sent);
        }
    }

    // ================================================
    // 位姿日志定时回调
    // ================================================
    void poseLogCallback()
    {
        if (pose_received_) {
            RCLCPP_INFO(this->get_logger(),
                "小车当前位置: x=%.2f, y=%.2f, theta=%.2f | 病虫害记录: %zu 条",
                current_x_, current_y_, current_theta_, pest_records_.size());
        } else {
            RCLCPP_WARN(this->get_logger(), "尚未收到 AMCL 位姿数据 (请确认 Nav2 + AMCL 已启动)");
        }
    }

    // ================================================
    // 工具函数
    // ================================================

    /**
     * @brief 四元数转偏航角 (弧度)
     */
    static double quaternionToYaw(const geometry_msgs::msg::Quaternion& q)
    {
        double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
        double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
        return std::atan2(siny_cosp, cosy_cosp);
    }

    /**
     * @brief 获取当前时间字符串 (ISO 8601)
     */
    static std::string getCurrentTimeStr()
    {
        auto now = std::chrono::system_clock::now();
        auto time_t = std::chrono::system_clock::to_time_t(now);
        std::ostringstream oss;
        oss << std::put_time(std::localtime(&time_t), "%Y-%m-%dT%H:%M:%S");
        return oss.str();
    }

    /**
     * @brief 构建病虫害报告 JSON 字符串 (用于 TCP 上传)
     */
    static std::string buildJsonReport(const PestRecord& record)
    {
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(4);
        oss << "{";
        oss << "\"type\":\"pest_report\",";
        oss << "\"pest_type\":" << static_cast<int>(record.pest_type) << ",";
        oss << "\"pest_type_name\":\"" << escapeJson(record.pest_type_name) << "\",";
        oss << "\"confidence\":" << record.confidence << ",";
        oss << "\"map_x\":" << record.map_x << ",";
        oss << "\"map_y\":" << record.map_y << ",";
        oss << "\"map_theta\":" << record.map_theta << ",";
        oss << "\"timestamp\":\"" << record.timestamp << "\",";
        oss << "\"image_path\":\"" << escapeJson(record.image_path) << "\"";
        oss << "}";
        return oss.str();
    }

    /**
     * @brief JSON 字符串转义
     */
    static std::string escapeJson(const std::string& s)
    {
        std::ostringstream oss;
        for (char c : s) {
            switch (c) {
                case '"':  oss << "\\\""; break;
                case '\\': oss << "\\\\"; break;
                case '\n': oss << "\\n";  break;
                case '\r': oss << "\\r";  break;
                case '\t': oss << "\\t";  break;
                default:   oss << c;      break;
            }
        }
        return oss.str();
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PestMonitor>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
