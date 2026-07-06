#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "cv_bridge/cv_bridge.h"

#include "image_geometry/pinhole_camera_model.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <opencv2/opencv.hpp>

#include <cmath>
#include <memory>

// Frame the object position is reported in: the arm's base link. The camera is
// a chain of fixed joints off `world`, so robot_state_publisher puts the whole
// transform on /tf_static and we can look it up at any time.
static constexpr char kTargetFrame[] = "robot_base";
static constexpr double kShoulder[3] = {0.0, 0.0, 0.18};
static constexpr double kReachMax = 0.85;
static constexpr double kReachMin = 0.15;

// Don't flood the arm: publish at most one target per this interval.
static const rclcpp::Duration kPublishPeriod = rclcpp::Duration::from_seconds(1.0);

// The object sits on the ground (z ~ 0), which the arm can't reach without
// driving the end-effector into the floor. Keep the detected x,y but command a
// fixed hover height above it in the robot_base frame. Tune to taste.
static constexpr double kHoverHeight = 0.7;

class VisionNode : public rclcpp::Node {
    public:
        VisionNode() : Node("vision_node") {
            cv::namedWindow("raw frame", cv::WINDOW_AUTOSIZE);
            cv::startWindowThread();

            tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
            tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

            // Intrinsics: build the pinhole model once from camera_info.
            info_subscriber_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
                "/camera/camera_info", 10,
                [this](const sensor_msgs::msg::CameraInfo::SharedPtr info) {
                    cam_model_.fromCameraInfo(*info);
                    have_model_ = true;
                });

            // Depth: cache the latest frame (32FC1, metres) for the rgb callback.
            depth_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
                "/camera/depth/image_raw", rclcpp::SensorDataQoS(),
                [this](const sensor_msgs::msg::Image::ConstSharedPtr msg) {
                    try {
                        latest_depth_ = cv_bridge::toCvCopy(msg, "32FC1")->image;
                    } catch (const cv_bridge::Exception & e) {
                        RCLCPP_ERROR(this->get_logger(), "depth cv_bridge: %s", e.what());
                    }
                });

            // RGB: where the actual work happens.
            image_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
                "/camera/image_raw", rclcpp::SensorDataQoS(),
                std::bind(&VisionNode::imageCallback, this, std::placeholders::_1));

            RCLCPP_INFO(this->get_logger(), "Vision node up, listening on /camera/image_raw");

            target_publisher_ = this->create_publisher<geometry_msgs::msg::PointStamped>("object_position", 10);

        }

    private:
        rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr target_publisher_;
        void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg) {
            cv::Mat frame;
            try {
                frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
            } catch (const cv_bridge::Exception & e) {
                RCLCPP_ERROR(this->get_logger(), "cv_bridge error: %s", e.what());
                return;
            }

            // --- 1. Segment the object ----------------------------------------
            // Background (ground + stand) is gray = low saturation; any object on
            // it is colored = high saturation. Keep high-saturation pixels and
            // take the largest blob as "the object".
            cv::Mat hsv, mask;
            cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
            cv::inRange(hsv, cv::Scalar(0, 80, 40), cv::Scalar(179, 255, 255), mask);
            cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, {3, 3});
            cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
            cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);

            std::vector<std::vector<cv::Point>> contours;
            cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

            int best = -1;
            double best_area = 50.0;  // ignore noise blobs smaller than this
            for (size_t i = 0; i < contours.size(); ++i) {
                double a = cv::contourArea(contours[i]);
                if (a > best_area) { best_area = a; best = static_cast<int>(i); }
            }

            if (best >= 0) {
                cv::Moments m = cv::moments(contours[best]);
                int u = static_cast<int>(m.m10 / m.m00);
                int v = static_cast<int>(m.m01 / m.m00);
                cv::drawContours(frame, contours, best, cv::Scalar(0, 255, 0), 2);
                cv::circle(frame, {u, v}, 4, cv::Scalar(0, 0, 255), -1);

                localize(u, v);  // 2-4: deproject + transform + log
            }

            cv::imshow("raw frame", frame);
            cv::waitKey(1);
        }

        // Deproject pixel (u,v) using depth + intrinsics, then transform into the
        // arm base frame and log it.
        void localize(int u, int v) {
            if (!have_model_) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                    "no camera_info yet");
                return;
            }
            if (latest_depth_.empty() ||
                u < 0 || v < 0 || u >= latest_depth_.cols || v >= latest_depth_.rows) {
                return;
            }

            // --- 2. Depth at the pixel (metres along the optical axis) ---------
            float d = latest_depth_.at<float>(v, u);
            if (!std::isfinite(d) || d <= 0.0f) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                    "no valid depth at (%d,%d)", u, v);
                return;
            }

            // --- 3. Deproject to a 3D point in the camera optical frame --------
            // projectPixelTo3dRay returns a ray with z scaled to 1; multiply by
            // depth so the point sits at the measured distance.
            cv::Point3d ray = cam_model_.projectPixelTo3dRay({static_cast<double>(u),
                                                              static_cast<double>(v)});
            geometry_msgs::msg::PointStamped p_cam;
            p_cam.header.frame_id = cam_model_.tfFrame();   // camera_optical_frame
            // stamp left at 0 => tf uses the latest available (static) transform
            p_cam.point.x = ray.x * d / ray.z;
            p_cam.point.y = ray.y * d / ray.z;
            p_cam.point.z = d;

            // --- 4. Transform into the arm base frame and log ------------------
            geometry_msgs::msg::PointStamped p_base;
            try {
                p_base = tf_buffer_->transform(p_cam, kTargetFrame,
                                               tf2::durationFromSec(0.2));
            } catch (const tf2::TransformException & e) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                    "tf %s -> %s failed: %s",
                    p_cam.header.frame_id.c_str(), kTargetFrame, e.what());
                return;
            }

            // --- 5. Reachability against the approximate workspace shell ------
            const double dx = p_base.point.x - kShoulder[0];
            const double dy = p_base.point.y - kShoulder[1];
            const double dz = p_base.point.z - kShoulder[2];
            const double r = std::sqrt(dx * dx + dy * dy + dz * dz);

            if (r >= kReachMin && r <= kReachMax) {
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                    "object in %s: x=%.3f y=%.3f z=%.3f -> REACHABLE",
                    kTargetFrame, p_base.point.x, p_base.point.y, p_base.point.z);

                publishTarget(p_base.point);
            } else {
                // Closest reachable point = clamp the object onto the shell along
                // the ray from the shoulder (nearest point on a sphere to an
                // outside/inside point lies on that radial line).
                const double target = (r > kReachMax) ? kReachMax : kReachMin;
                const double s = (r > 1e-6) ? target / r : 0.0;
                geometry_msgs::msg::Point clamped;
                clamped.x = kShoulder[0] + dx * s;
                clamped.y = kShoulder[1] + dy * s;
                clamped.z = kShoulder[2] + dz * s;
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                    "object in %s: x=%.3f y=%.3f z=%.3f -> UNREACHABLE (dist=%.3f m, "
                    "shell=[%.2f,%.2f]); sending closest reachable: x=%.3f y=%.3f z=%.3f",
                    kTargetFrame, p_base.point.x, p_base.point.y, p_base.point.z,
                    r, kReachMin, kReachMax, clamped.x, clamped.y, clamped.z);

                // Publish the clamped point, not the raw (out-of-reach) one, so
                // the arm is always given something it can actually try to reach.
                publishTarget(clamped);
            }
        }

        // Publish a target, rate-limited to kPublishPeriod. The vision callbacks
        // run at camera rate (~30 Hz) but the arm only needs an occasional goal;
        // throttling here keeps the topic (and the arm's logs) sane.
        void publishTarget(const geometry_msgs::msg::Point & point) {
            const rclcpp::Time now = this->get_clock()->now();
            if (last_publish_time_.nanoseconds() != 0 &&
                (now - last_publish_time_) < kPublishPeriod) {
                return;
            }
            last_publish_time_ = now;

            geometry_msgs::msg::PointStamped target_msg;
            target_msg.header.stamp = now;
            target_msg.header.frame_id = kTargetFrame;
            target_msg.point = point;
            target_msg.point.z = kHoverHeight;  // hover above the object, don't dive to the floor
            target_publisher_->publish(target_msg);
        }

        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber_;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_subscriber_;
        rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_subscriber_;

        image_geometry::PinholeCameraModel cam_model_;
        bool have_model_ = false;
        cv::Mat latest_depth_;

        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

        rclcpp::Time last_publish_time_{0, 0, RCL_ROS_TIME};
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VisionNode>());
    rclcpp::shutdown();
    return 0;
}
