#include "mask/AnimalFilter.hpp"
#include <cmath>
#include <algorithm>
#include "ament_index_cpp/get_package_share_directory.hpp"

AnimalFilter::AnimalFilter(const std::string& animal_name) {
    load_masks(animal_name);
}

void AnimalFilter::load_masks(const std::string& animal_name) {
    std::string base_path = ament_index_cpp::get_package_share_directory("coco_filters") + "/imgs/animals_mask/" + animal_name + "/";
    
    mask_closed_closed_ = cv::imread(base_path + animal_name + "_closed_closed.png", cv::IMREAD_UNCHANGED);
    mask_open_closed_ = cv::imread(base_path + animal_name + "_open_closed.png", cv::IMREAD_UNCHANGED);
    mask_closed_open_ = cv::imread(base_path + animal_name + "_closed_open.png", cv::IMREAD_UNCHANGED);
    mask_open_open_ = cv::imread(base_path + animal_name + "_open_open.png", cv::IMREAD_UNCHANGED);

    if(mask_closed_closed_.empty() || mask_open_closed_.empty() || 
       mask_closed_open_.empty() || mask_open_open_.empty()) {
        throw std::runtime_error("No se pudieron cargar las máscaras del animal");
    }
}

cv::Mat AnimalFilter::rotate_image(const cv::Mat& image, double angle) {
    cv::Point2f center(image.cols/2.0f, image.rows/2.0f);
    cv::Mat rot = cv::getRotationMatrix2D(center, angle, 1.0);
    
    cv::Rect2f bbox = cv::RotatedRect(cv::Point2f(), image.size(), angle).boundingRect2f();
    rot.at<double>(0,2) += bbox.width/2.0 - image.cols/2.0;
    rot.at<double>(1,2) += bbox.height/2.0 - image.rows/2.0;

    cv::Mat rotated;
    cv::warpAffine(image, rotated, rot, bbox.size(), 
                  cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0,0,0,0));
    return rotated;
}

void AnimalFilter::optimized_overlay(cv::Mat& bg, const cv::Mat& overlay, int x, int y) {
    if (overlay.empty() || bg.empty() ||
        x >= bg.cols || y >= bg.rows ||
        x + overlay.cols <= 0 || y + overlay.rows <= 0) {
        return;
    }

    int y1 = std::max(y, 0);
    int y2 = std::min(y + overlay.rows, bg.rows);
    int x1 = std::max(x, 0);
    int x2 = std::min(x + overlay.cols, bg.cols);

    if (x1 >= x2 || y1 >= y2) return;

    cv::Mat bg_3channel;
    if (bg.channels() == 4) {
        cv::cvtColor(bg, bg_3channel, cv::COLOR_BGRA2BGR);
    } else if (bg.channels() == 3) {
        bg_3channel = bg.clone();
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("AnimalFilter"), "Formato no soportado: %d canales", bg.channels());
        return;
    }

    cv::Rect roi(x1, y1, x2 - x1, y2 - y1);
    cv::Mat bg_roi = bg_3channel(roi);

    cv::Rect overlay_roi(x1 - x, y1 - y, x2 - x1, y2 - y1);
    cv::Mat overlay_region = overlay(overlay_roi);

    std::vector<cv::Mat> overlay_channels;
    cv::split(overlay_region, overlay_channels);

    if(overlay_channels.size() < 4) {
        RCLCPP_ERROR(rclcpp::get_logger("AnimalFilter"), "La máscara no tiene canal alpha");
        return;
    }

    cv::Mat alpha;
    overlay_channels[3].convertTo(alpha, CV_32F, 1.0/255.0);
    cv::Mat inv_alpha = 1.0 - alpha;

    std::vector<cv::Mat> overlay_color_channels(3);
    for(int i = 0; i < 3; ++i) {
        overlay_channels[i].convertTo(overlay_color_channels[i], CV_32F);
    }

    std::vector<cv::Mat> bg_channels;
    cv::split(bg_roi, bg_channels);
    for(auto& channel : bg_channels) channel.convertTo(channel, CV_32F);

    for(int c = 0; c < 3; ++c) {
        cv::add(
            bg_channels[c].mul(inv_alpha),
            overlay_color_channels[c].mul(alpha),
            bg_channels[c]
        );
    }

    cv::merge(bg_channels, bg_roi);
    bg_roi.convertTo(bg_roi, CV_8U);

    bg_roi.copyTo(bg(roi));
}

float AnimalFilter::smooth_value(std::deque<float>& history, float new_value, size_t max_size) {
    history.push_back(new_value);
    if(history.size() > max_size) history.pop_front();
    
    float sum = 0.0f;
    for(auto& val : history) sum += val;
    return sum / history.size();
}

cv::Mat AnimalFilter::apply_filter(cv::Mat frame, const std::vector<cv::Point2f>& landmarks) {
    if(landmarks.size() < 400) return frame;

    try {
        const int LEFT_EYE = 33;
        const int RIGHT_EYE = 263;
        const int NOSE_TIP = 1;
        const int CHIN = 152;
        const int FOREHEAD = 10;
        const int LIP_TOP = 13;
        const int LIP_BOTTOM = 14;
        const int LEFT_EYE_TOP = 159;
        const int LEFT_EYE_BOTTOM = 145;
        const int RIGHT_EYE_TOP = 386;
        const int RIGHT_EYE_BOTTOM = 374;

        cv::Point2f left_eye = landmarks[LEFT_EYE];
        cv::Point2f right_eye = landmarks[RIGHT_EYE];
        cv::Point2f nose = landmarks[NOSE_TIP];
        cv::Point2f chin = landmarks[CHIN];
        cv::Point2f forehead = landmarks[FOREHEAD];

        float face_width = cv::norm(right_eye - left_eye) * 3.2f;
        float face_height = cv::norm(forehead - chin) * 2.0f;

        float dx = right_eye.x - left_eye.x;
        float dy = right_eye.y - left_eye.y;
        double angle = -std::atan2(dy, dx) * 180.0 / CV_PI;

        cv::Point2f lip_top = landmarks[LIP_TOP];
        cv::Point2f lip_bottom = landmarks[LIP_BOTTOM];
        float mouth_distance = cv::norm(lip_top - lip_bottom);
        bool mouth_open = smooth_value(mouth_open_history_, mouth_distance > 12.0f, 5) > 0.5f;

        cv::Point2f left_eye_top = landmarks[LEFT_EYE_TOP];
        cv::Point2f left_eye_bottom = landmarks[LEFT_EYE_BOTTOM];
        cv::Point2f right_eye_top = landmarks[RIGHT_EYE_TOP];
        cv::Point2f right_eye_bottom = landmarks[RIGHT_EYE_BOTTOM];
        
        float eye_openness = (cv::norm(left_eye_top - left_eye_bottom) + 
                             cv::norm(right_eye_top - right_eye_bottom)) / 2.0f;
        bool eyes_open = smooth_value(eyes_open_history_, eye_openness > 5.0f, 5) > 0.5f;

        cv::Mat selected_mask;
        if(mouth_open && eyes_open) selected_mask = mask_open_open_;
        else if(mouth_open) selected_mask = mask_closed_open_;
        else if(eyes_open) selected_mask = mask_open_closed_;
        else selected_mask = mask_closed_closed_;

        cv::Mat resized_mask;
        cv::resize(selected_mask, resized_mask, cv::Size(face_width, face_height));
        
        cv::Mat rotated_mask = rotate_image(resized_mask, angle);
        
        int pos_x = static_cast<int>(nose.x - rotated_mask.cols / 2.0f);
        int pos_y = static_cast<int>(forehead.y - rotated_mask.rows * 0.3f);

        optimized_overlay(frame, rotated_mask, pos_x, pos_y);

    } catch(const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("AnimalFilter"), "Error: %s", e.what());
    }
    
    return frame;
}