#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <stdexcept>
#include <algorithm>
#include <cctype>
#include <random>
#include <numeric>

#include <opencv2/opencv.hpp>
#include <opencv2/ml.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

// --- HELPER FUNCTIONS (Must be above main) ---

std::string trim(const std::string& str) {
    size_t first = str.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) return "";
    size_t last = str.find_last_not_of(" \t\r\n");
    return str.substr(first, last - first + 1);
}

std::string to_lowercase(std::string str) {
    std::transform(str.begin(), str.end(), str.begin(),
                   [](unsigned char c) { return std::tolower(c); });
    return str;
}

int parse_label(const std::string& raw_label) {
    std::string label = to_lowercase(trim(raw_label));
    return (label == "recyclable") ? 1 : 0;
}

void load_dataset(const std::string& filename, cv::Mat& data, cv::Mat& labels) {
    std::ifstream file(filename);
    if (!file.is_open()) throw std::runtime_error("Could not open file: " + filename);

    std::string line;
    std::vector<float> feature_vec;
    std::vector<int> label_vec;

    std::getline(file, line); // Skip header
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string value;
        std::vector<std::string> row;
        while (std::getline(ss, value, ',')) row.push_back(value);

        if (row.size() < 7) continue;

        try {
            for (int i = 1; i <= 5; ++i) {
                feature_vec.push_back(std::stof(trim(row[i])));
            }
            label_vec.push_back(parse_label(row[6]));
        } catch (...) { continue; }
    }

    data = cv::Mat(static_cast<int>(label_vec.size()), 5, CV_32F, feature_vec.data()).clone();
    labels = cv::Mat(static_cast<int>(label_vec.size()), 1, CV_32S, label_vec.data()).clone();
}

void compute_feature_scaling(const cv::Mat& data, cv::Mat& mean, cv::Mat& stddev) {
    mean = cv::Mat(1, data.cols, CV_32F);
    stddev = cv::Mat(1, data.cols, CV_32F);
    for (int col = 0; col < data.cols; ++col) {
        cv::Scalar m, s;
        cv::meanStdDev(data.col(col), m, s);
        mean.at<float>(0, col) = static_cast<float>(m[0]);
        stddev.at<float>(0, col) = (static_cast<float>(s[0]) < 1e-6f) ? 1.0f : static_cast<float>(s[0]);
    }
}

cv::Mat scale_features(const cv::Mat& data, const cv::Mat& mean, const cv::Mat& stddev) {
    cv::Mat scaled(data.rows, data.cols, CV_32F);
    for (int r = 0; r < data.rows; ++r) {
        for (int c = 0; c < data.cols; ++c) {
            scaled.at<float>(r, c) = (data.at<float>(r, c) - mean.at<float>(0, c)) / stddev.at<float>(0, c);
        }
    }
    return scaled;
}

float evaluate_model(const cv::Ptr<cv::ml::SVM>& svm, const cv::Mat& data, const cv::Mat& labels) {
    int correct = 0;
    for (int i = 0; i < data.rows; ++i) {
        if (static_cast<int>(svm->predict(data.row(i))) == labels.at<int>(i, 0)) correct++;
    }
    return 100.0f * static_cast<float>(correct) / static_cast<float>(data.rows);
}

// --- MAIN EXECUTION ---

int main(int, char**) {
    try {
        std::string pkg_share = ament_index_cpp::get_package_share_directory("bottle_detector");
        std::string csv_path = pkg_share + "/data/increased_recycling_dataset.csv";

        cv::Mat raw_data, raw_labels;
        std::cout << "Loading dataset..." << std::endl;
        load_dataset(csv_path, raw_data, raw_labels);

        // 1. Shuffle
        std::vector<int> indices(raw_data.rows);
        std::iota(indices.begin(), indices.end(), 0);
        std::random_device rd;
        std::mt19937 g(rd());
        // std::mt19937 g(42); // Uses '42' as a constant seed for reproducibility
        std::shuffle(indices.begin(), indices.end(), g);

        cv::Mat shuffled_data(raw_data.rows, raw_data.cols, raw_data.type());
        cv::Mat shuffled_labels(raw_labels.rows, raw_labels.cols, raw_labels.type());
        for (int i = 0; i < raw_data.rows; ++i) {
            raw_data.row(indices[i]).copyTo(shuffled_data.row(i));
            raw_labels.row(indices[i]).copyTo(shuffled_labels.row(i));
        }

        // 2. Split (80/20)
        int train_rows = static_cast<int>(shuffled_data.rows * 0.8);
        cv::Mat train_data_raw = shuffled_data.rowRange(0, train_rows);
        cv::Mat train_labels = shuffled_labels.rowRange(0, train_rows);
        cv::Mat test_data_raw = shuffled_data.rowRange(train_rows, shuffled_data.rows);
        cv::Mat test_labels = shuffled_labels.rowRange(train_rows, shuffled_data.rows);

        // 3. Scale
        cv::Mat mean, stddev;
        compute_feature_scaling(train_data_raw, mean, stddev);
        cv::Mat train_scaled = scale_features(train_data_raw, mean, stddev);
        cv::Mat test_scaled = scale_features(test_data_raw, mean, stddev);

        // // Save scaling
        // cv::FileStorage fs("scaling_params.xml", cv::FileStorage::WRITE);
        // fs << "mean" << mean << "stddev" << stddev;
        // fs.release();

        // 4. Train
        cv::Ptr<cv::ml::SVM> svm = cv::ml::SVM::create();
        svm->setType(cv::ml::SVM::C_SVC);
        svm->setKernel(cv::ml::SVM::RBF);
        svm->setGamma(0.1);
        svm->setC(10.0);
        svm->train(train_scaled, cv::ml::ROW_SAMPLE, train_labels);

        // 5. Accuracy
        std::cout << "Train Accuracy: " << evaluate_model(svm, train_scaled, train_labels) << "%" << std::endl;
        std::cout << "Test Accuracy: " << evaluate_model(svm, test_scaled, test_labels) << "%" << std::endl;

        // svm->save("bottle_svm_model.xml");
        // std::cout << "SUCCESS: Model and scaling saved." << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "ERROR: " << e.what() << std::endl;
        return 1;
    }
    return 0;
}