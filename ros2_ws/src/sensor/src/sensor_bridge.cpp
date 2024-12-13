#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"

class SensorBridge : public rclcpp::Node
{
public:
    SensorBridge(std::string name);

private:
    // 彩色图像转发
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr color_publisher;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr color_subscription;

    // IMU Publisher
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher;

    // IMU Subscriptions (with message filters)
    message_filters::Subscriber<sensor_msgs::msg::Imu> accel_subscriber;
    message_filters::Subscriber<sensor_msgs::msg::Imu> gyro_subscriber;

    // TimeSynchronizer to synchronize both accel and gyro data
    message_filters::TimeSynchronizer<sensor_msgs::msg::Imu, sensor_msgs::msg::Imu> sync;

    void color_callback(const sensor_msgs::msg::Image::SharedPtr msg);
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr accel_msg, const sensor_msgs::msg::Imu::SharedPtr gyro_msg);
};

SensorBridge::SensorBridge(std::string name)
    : Node(name),
    accel_subscriber(this, "/camera/accel/sample"), // 使用普通对象来初始化订阅者
      gyro_subscriber(this, "/camera/gyro/sample"),   // 同上
      sync(accel_subscriber, gyro_subscriber, 10)     // 使用普通对象初始化同步器
{
    RCLCPP_INFO(this->get_logger(), "%s has started", name.c_str());

    // 创建 color 图像的 publisher 和 subscription
    color_publisher = this->create_publisher<sensor_msgs::msg::Image>("/cam0/image_raw", rclcpp::QoS(10));
    color_subscription = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/color/image_raw", 
        rclcpp::QoS(10),
        std::bind(&SensorBridge::color_callback, this, std::placeholders::_1)
    );

    // 创建 IMU 数据的 publisher
    imu_publisher = this->create_publisher<sensor_msgs::msg::Imu>("/imu0", rclcpp::QoS(10));

    // accel_subscriber.subscribe(this, "/camera/accel/sample");
    // gyro_subscriber.subscribe(this, "/camera/gyro/sample");


    // // 创建同步器来同步加速度计和陀螺仪数据
    // sync(accel_subscriber, gyro_subscriber, 10);
    sync.registerCallback(&SensorBridge::imu_callback, this);
}

void SensorBridge::color_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    color_publisher->publish(*msg);
    RCLCPP_INFO(this->get_logger(), "Color Image Published");
}

void SensorBridge::imu_callback(const sensor_msgs::msg::Imu::SharedPtr accel_msg, const sensor_msgs::msg::Imu::SharedPtr gyro_msg)
{
    // 合并加速度计和陀螺仪数据到一个IMU消息
    sensor_msgs::msg::Imu imu_msg;

    // 设置 header 和时间戳
    imu_msg.header.stamp = accel_msg->header.stamp;
    imu_msg.header.frame_id = "imu_link"; // 设置合适的 frame_id

    // 组合加速度数据
    imu_msg.linear_acceleration = accel_msg->linear_acceleration;
    imu_msg.linear_acceleration_covariance = accel_msg->linear_acceleration_covariance;

    // 组合角速度数据
    imu_msg.angular_velocity = gyro_msg->angular_velocity;
    imu_msg.angular_velocity_covariance = gyro_msg->angular_velocity_covariance;

    // 设置姿态数据
    // 设置默认的 orientation 和 orientation_covariance
    imu_msg.orientation.x = 0.0;
    imu_msg.orientation.y = 0.0;
    imu_msg.orientation.z = 0.0;
    imu_msg.orientation.w = 1.0;

    imu_msg.orientation_covariance = {
        99999.9, 0.0, 0.0,
        0.0, 99999.9, 0.0,
        0.0, 0.0, 99999.9
    };

    // 发布合并后的IMU数据
    imu_publisher->publish(imu_msg);
    RCLCPP_INFO(this->get_logger(), "IMU Data Published");
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SensorBridge>("sensor_bridge");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
