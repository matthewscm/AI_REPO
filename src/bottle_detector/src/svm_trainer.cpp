#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <opencv2/opencv.hpp>
#include <opencv2/ml.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

// Helper to read CSV into OpenCV Mats
void load_dataset(const std::string& filename, cv::Mat& data, cv::Mat& labels) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        throw std::runtime_error("Could not open file: " + filename);
    }

    std::string line;
    std::vector<float> feature_vec;
    std::vector<int> label_vec;

    std::getline(file, line); // Skip header

    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string value;
        std::vector<std::string> row;

        while (std::getline(ss, value, ',')) {
            row.push_back(value);
        }

        if (row.size() < 7) continue;

        // Features: Height, Width, H, S, V (Indices 1-5)
        try {
            for (int i = 1; i <= 5; ++i) {
                feature_vec.push_back(std::stof(row[i]));
            }
            // Labels: "Recyclable" -> 1, else 0
            label_vec.push_back((row[6] == "Recyclable") ? 1 : 0);
        } catch (...) {
            continue; // Skip rows with bad numeric data
        }
    }

    data = cv::Mat(feature_vec.size() / 5, 5, CV_32F, feature_vec.data()).clone();
    labels = cv::Mat(label_vec.size(), 1, CV_32S, label_vec.data()).clone();
}

int main(int, char**) {
    try {
        // --- Use ament_index_cpp to find the data file ---
        std::string pkg_share = ament_index_cpp::get_package_share_directory("bottle_detector");
        std::string csv_path = pkg_share + "/data/increased_recycling_dataset.csv";
        
        cv::Mat training_data, labels;

        std::cout << "Loading dataset: " << csv_path << "..." << std::endl;
        load_dataset(csv_path, training_data, labels);

        if (training_data.empty()) {
            throw std::runtime_error("Dataset is empty. Check your CSV content.");
        }

        // --- Step 3: Feature Scaling ---
        cv::Mat mean, stddev;
        cv::meanStdDev(training_data, mean, stddev);
        
        // Save scaling parameters (saved to current directory where you run the node)
        cv::FileStorage fs_scaling("scaling_params.xml", cv::FileStorage::WRITE);
        fs_scaling << "mean" << mean;
        fs_scaling << "stddev" << stddev;
        fs_scaling.release();

        cv::Mat training_data_scaled;
        for (int i = 0; i < training_data.rows; ++i) {
            cv::Mat row = (training_data.row(i) - mean.t()) / stddev.t();
            training_data_scaled.push_back(row);
        }

        // --- Step 4 & 5: SVM Setup and Training ---
        cv::Ptr<cv::ml::SVM> svm = cv::ml::SVM::create();
        svm->setType(cv::ml::SVM::C_SVC);
        svm->setKernel(cv::ml::SVM::RBF);
        svm->setGamma(0.01); 
        svm->setC(1.0);
        svm->setTermCriteria(cv::TermCriteria(cv::TermCriteria::MAX_ITER, 100, 1e-6));

        std::cout << "Training SVM... samples: " << training_data.rows << std::endl;
        svm->train(training_data_scaled, cv::ml::ROW_SAMPLE, labels);

        // Save the model
        svm->save("bottle_svm_model.xml");
        std::cout << "------------------------------------------" << std::endl;
        std::cout << "SUCCESS: Model saved to bottle_svm_model.xml" << std::endl;
        std::cout << "SUCCESS: Scaling saved to scaling_params.xml" << std::endl;
        std::cout << "------------------------------------------" << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "ERROR: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}