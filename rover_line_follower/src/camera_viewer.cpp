#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class CameraViewer : public rclcpp::Node
{
public:
  CameraViewer()
  : Node("camera_viewer")
  {
    auto qos_profile = rclcpp::SensorDataQoS();

    // 1. Subscriber: 원본 이미지 받기
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/image_raw",
      qos_profile,
      std::bind(&CameraViewer::image_callback, this, std::placeholders::_1));

    // 2. Publisher: 뒤집힌 이미지 내보내기 (토픽명: camera/image_flipped)
    publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
      "camera/image_flipped",
      10 // Queue Size
    );

    RCLCPP_INFO(this->get_logger(), "Camera Flipper Node Started.");
    RCLCPP_INFO(this->get_logger(), "Sub: /image_raw -> Pub: /camera/image_flipped");
  }

private:
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    try {
      // 빈 이미지 체크 (카메라 문제로 인한 빈 이미지 처리)
      if (msg->width == 0 || msg->height == 0 || msg->data.empty()) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
                            "Received empty image (width=%d, height=%d, data_size=%zu), skipping...",
                            msg->width, msg->height, msg->data.size());
        return;
      }

      // A. ROS -> OpenCV 변환
      cv_bridge::CvImagePtr cv_ptr;
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

      // 변환된 이미지가 비어있는지 확인
      if (cv_ptr->image.empty() || cv_ptr->image.rows == 0 || cv_ptr->image.cols == 0) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                            "Converted image is empty, skipping...");
        return;
      }

      // B. 이미지 180도 회전
      cv::Mat flipped_image;
      cv::flip(cv_ptr->image, flipped_image, -1);

      // C. OpenCV -> ROS 메시지 변환 및 발행
      // 변환을 위해 새로운 cv_bridge 객체를 만듭니다.
      cv_bridge::CvImage out_msg;

      // [중요] 원본 메시지의 헤더(시간, frame_id 등)를 그대로 유지해야 TF 등에서 문제가 안 생깁니다.
      out_msg.header = msg->header;
      out_msg.encoding = sensor_msgs::image_encodings::BGR8;
      out_msg.image = flipped_image;

      // D. 토픽 발행
      publisher_->publish(*out_msg.toImageMsg());

    } catch (cv_bridge::Exception& e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_; // 퍼블리셔 추가
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraViewer>());
  rclcpp::shutdown();
  return 0;
}
