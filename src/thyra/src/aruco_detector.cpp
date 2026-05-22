#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "std_msgs/msg/float64.hpp"
#include "interfaces/msg/drone_state.hpp"
#include <cv_bridge/cv_bridge.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <iostream>
#include <cmath>
#include <vector>

using std::placeholders::_1;

class ArucoDetectorCpp : public rclcpp::Node {
public:
    ArucoDetectorCpp() : Node("aruco_detector") {
        // Parameters
        this->declare_parameter<bool>("show_window", true);
        show_window_ = this->get_parameter("show_window").as_bool();

        drone_pos_ = {0.0, 0.0, 0.0};
        drone_att_ = {0.0, 0.0, 0.0};
        mount_pitch_ = 45.0 * M_PI / 180.0;
        gimbal_pitch_ = -999.0;

        w_ = 640.0;
        h_ = 480.0;
        double hfov_rad = 85.0 * M_PI / 180.0;
        f_px_ = (w_ / 2.0) / std::tan(hfov_rad / 2.0);

        aruco_dict_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
        aruco_params_ = cv::aruco::DetectorParameters::create();
        aruco_params_->minMarkerPerimeterRate = 0.01;
        aruco_params_->perspectiveRemovePixelPerCell = 4;

        pub_error_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>(
            "/asr/aruco/pixel_error", 1);

        // Annotated detector overlay — viewable remotely via rqt_image_view
        pub_annotated_ = this->create_publisher<sensor_msgs::msg::Image>(
            "/asr/aruco/detector_image", rclcpp::SensorDataQoS());

        auto qos_sensor = rclcpp::SensorDataQoS();

        sub_image_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/camera/color/image_raw", qos_sensor,
            std::bind(&ArucoDetectorCpp::image_cb, this, _1));

        sub_drone_ = this->create_subscription<interfaces::msg::DroneState>(
            "/asr/thyra/out/drone_state", 10,
            std::bind(&ArucoDetectorCpp::drone_cb, this, _1));

        sub_gimbal_ = this->create_subscription<std_msgs::msg::Float64>(
            "/gimbal/cmd_pitch", 10,
            std::bind(&ArucoDetectorCpp::gimbal_cb, this, _1));

        RCLCPP_INFO(this->get_logger(),
                    "C++ ArUco detector started  show_window=%s",
                    show_window_ ? "true" : "false");
    }

private:
    bool show_window_;
    std::vector<double> drone_pos_;
    std::vector<double> drone_att_;
    double mount_pitch_;
    double gimbal_pitch_;
    double w_, h_, f_px_;

    cv::Ptr<cv::aruco::Dictionary> aruco_dict_;
    cv::Ptr<cv::aruco::DetectorParameters> aruco_params_;

    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr pub_error_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_annotated_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_image_;
    rclcpp::Subscription<interfaces::msg::DroneState>::SharedPtr sub_drone_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_gimbal_;

    void drone_cb(const interfaces::msg::DroneState::SharedPtr msg) {
        if (msg->position.size() >= 3) {
            drone_pos_[0] = msg->position[0];
            drone_pos_[1] = msg->position[1];
            drone_pos_[2] = msg->position[2];
        }
        if (msg->orientation.size() >= 3) {
            drone_att_[0] = msg->orientation[0];
            drone_att_[1] = msg->orientation[1];
            drone_att_[2] = msg->orientation[2];
        }
    }

    void gimbal_cb(const std_msgs::msg::Float64::SharedPtr msg) {
        gimbal_pitch_ = (msg->data + 1.0) * (M_PI / 4.0);
    }

    cv::Mat rot(double r, double p, double y) {
        double cr = std::cos(r), sr = std::sin(r);
        double cp = std::cos(p), sp = std::sin(p);
        double cy = std::cos(y), sy = std::sin(y);

        cv::Mat Rz = (cv::Mat_<double>(3, 3) << cy, -sy, 0, sy, cy, 0, 0, 0, 1);
        cv::Mat Ry = (cv::Mat_<double>(3, 3) << cp, 0, sp, 0, 1, 0, -sp, 0, cp);
        cv::Mat Rx = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, cr, -sr, 0, sr, cr);

        return Rz * Ry * Rx;
    }

    bool to_ground(double u, double v, double alt, double& out_x, double& out_y) {
        cv::Mat vec_c = (cv::Mat_<double>(3, 1) << -(v - h_ / 2.0) / f_px_,
                                                    (u - w_ / 2.0) / f_px_,
                                                    1.0);

        double pitch_total = (gimbal_pitch_ > -900.0) ? gimbal_pitch_ : mount_pitch_;

        cv::Mat R_drone = rot(drone_att_[0], drone_att_[1], drone_att_[2]);
        cv::Mat R_mount = rot(0.0, pitch_total, 0.0);
        cv::Mat R = R_drone * R_mount;

        cv::Mat vec_n_target = R * vec_c;
        if (vec_n_target.at<double>(2, 0) <= 0) return false;
        double k_target = alt / vec_n_target.at<double>(2, 0);
        cv::Mat ground_target = vec_n_target * k_target;

        cv::Mat vec_c_center = (cv::Mat_<double>(3, 1) << 0.0, 0.0, 1.0);
        cv::Mat vec_n_center = R * vec_c_center;
        if (vec_n_center.at<double>(2, 0) <= 0) return false;
        double k_center = alt / vec_n_center.at<double>(2, 0);
        cv::Mat ground_center = vec_n_center * k_center;

        out_x = ground_target.at<double>(0, 0) - ground_center.at<double>(0, 0);
        out_y = ground_target.at<double>(1, 0) - ground_center.at<double>(1, 0);
        return true;
    }

    bool to_ground_absolute(double u, double v, double alt, double& out_n, double& out_e) {
        cv::Mat vec_c = (cv::Mat_<double>(3, 1) << -(v - h_ / 2.0) / f_px_,
                                                    (u - w_ / 2.0) / f_px_,
                                                    1.0);

        double pitch_total = (gimbal_pitch_ > -900.0) ? gimbal_pitch_ : mount_pitch_;

        cv::Mat R_drone = rot(drone_att_[0], drone_att_[1], drone_att_[2]);
        cv::Mat R_mount = rot(0.0, pitch_total, 0.0);
        cv::Mat R = R_drone * R_mount;

        cv::Mat vec_n = R * vec_c;
        if (vec_n.at<double>(2, 0) <= 0) return false;
        double k = alt / vec_n.at<double>(2, 0);
        
        out_n = vec_n.at<double>(0, 0) * k;
        out_e = vec_n.at<double>(1, 0) * k;
        return true;
    }

    void image_cb(const sensor_msgs::msg::Image::SharedPtr msg) {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        cv::Mat frame = cv_ptr->image;
        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners;

        cv::aruco::detectMarkers(frame, aruco_dict_, corners, ids, aruco_params_);

        geometry_msgs::msg::Vector3Stamped out;
        out.header.stamp = this->now();

        bool found = false;
        int idx = -1;
        for (size_t i = 0; i < ids.size(); i++) {
            if (ids[i] == 0) {
                found = true;
                idx = i;
                break;
            }
        }

        if (found) {
            std::vector<cv::Point2f> c = corners[idx];
            double mc_x = 0, mc_y = 0;
            for (auto p : c) {
                mc_x += p.x;
                mc_y += p.y;
            }
            mc_x /= 4.0;
            mc_y /= 4.0;

            double alt = -drone_pos_[2];
            double err_x_m, err_y_m;
            if (to_ground(mc_x, mc_y, alt, err_x_m, err_y_m)) {
                out.vector.x = err_x_m;
                out.vector.y = err_y_m;
                out.vector.z = 1.0;
            } else {
                out.vector.x = 0.0;
                out.vector.y = 0.0;
                out.vector.z = 0.0;
            }

            double mid_front_u = (c[0].x + c[1].x) / 2.0;
            double mid_front_v = (c[0].y + c[1].y) / 2.0;
            double mid_back_u = (c[3].x + c[2].x) / 2.0;
            double mid_back_v = (c[3].y + c[2].y) / 2.0;

            double gp_front_n, gp_front_e;
            double gp_back_n, gp_back_e;
            double relative_yaw_deg = 0.0;

            if (to_ground_absolute(mid_front_u, mid_front_v, alt, gp_front_n, gp_front_e) &&
                to_ground_absolute(mid_back_u, mid_back_v, alt, gp_back_n, gp_back_e)) {
                double fwd_n = gp_front_n - gp_back_n;
                double fwd_e = gp_front_e - gp_back_e;
                double marker_heading_world = std::atan2(fwd_e, fwd_n);
                double drone_yaw = drone_att_[2];
                relative_yaw_deg = (marker_heading_world - drone_yaw) * 180.0 / M_PI;
                relative_yaw_deg = std::fmod(relative_yaw_deg + 180.0, 360.0);
                if (relative_yaw_deg < 0) relative_yaw_deg += 360.0;
                relative_yaw_deg -= 180.0;
            }

            char buf[32];
            snprintf(buf, sizeof(buf), "%.2f", relative_yaw_deg);
            out.header.frame_id = buf;

            cv::aruco::drawDetectedMarkers(frame, corners, ids);
            cv::circle(frame, cv::Point(std::round(mc_x), std::round(mc_y)), 6, cv::Scalar(0, 255, 0), -1);

        } else {
            out.vector.x = 0.0;
            out.vector.y = 0.0;
            out.vector.z = 0.0;
            out.header.frame_id = "0.0";
        }

        try {
            pub_error_->publish(out);
        } catch (...) {
            // Context invalid on shutdown
        }

        // Republish annotated frame so it can be viewed remotely (rqt_image_view)
        try {
            std_msgs::msg::Header hdr;
            hdr.stamp = this->now();
            hdr.frame_id = "aruco_detector";
            auto out_img = cv_bridge::CvImage(hdr, "bgr8", frame).toImageMsg();
            pub_annotated_->publish(*out_img);
        } catch (...) {
            // Context invalid on shutdown
        }

        if (show_window_) {
            cv::imshow("ArUco Detector", frame);
            cv::waitKey(1);
        }
    }
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ArucoDetectorCpp>();
    rclcpp::spin(node);
    try { cv::destroyAllWindows(); } catch (...) {}  // No-op if no windows opened
    rclcpp::shutdown();
    return 0;
}
