#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <opencv2/opencv.hpp>
#include <opencv2/ml.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

int main(int, char**) {
    try {
        // Find the package share directory
        std::string pkg_share = ament_index_cpp::get_package_share_directory("bottle_detector");
        
        // --- 1. Load the Scaling Parameters ---
        cv::Mat mean, stddev;
        std::string scaling_path = pkg_share + "/models/scaling_params.xml";
        cv::FileStorage fs(scaling_path, cv::FileStorage::READ);
        
        if (!fs.isOpened()) {
            throw std::runtime_error("Could not find scaling file at: " + scaling_path);
        }
        fs["mean"] >> mean;
        fs["stddev"] >> stddev;
        fs.release();

        // --- 2. Load the SVM Model ---
        std::string model_path = pkg_share + "/models/bottle_svm_model.xml";
        cv::Ptr<cv::ml::SVM> svm = cv::ml::SVM::load(model_path);
        
        if (svm.empty()) {
            throw std::runtime_error("Could not load SVM model at: " + model_path);
        }

        // --- 3. Load the TEST dataset ---
        std::string test_csv = pkg_share + "/data/test_sorting_data.csv";
        std::ifstream file(test_csv);
        if (!file.is_open()) {
            throw std::runtime_error("Could not open test file: " + test_csv);
        }

        std::string line;
        std::getline(file, line); // Skip header

        int correct = 0;
        int total = 0;

        std::cout << "\n--- SVM TEST RESULTS ---" << std::endl;
        std::cout << "Using Model: " << model_path << std::endl;
        
        while (std::getline(file, line)) {
            std::stringstream ss(line);
            std::string val;
            std::vector<std::string> row;
            while (std::getline(ss, val, ',')) row.push_back(val);

            // Expecting 6 columns: Height, Width, H, S, V, Label
            if (row.size() < 6) continue;

            // Extract features (Indices 0-4)
            cv::Mat sample = (cv::Mat_<float>(1,5) << 
                std::stof(row[0]), std::stof(row[1]), std::stof(row[2]), std::stof(row[3]), std::stof(row[4]));
            
            // Extract true label (Index 5)
            int true_label = (row[5] == "Recyclable") ? 1 : 0;

            // --- 4. SCALE and PREDICT ---
            // Apply scaling using the loaded mean and stddev
            cv::Mat scaled_sample = (sample - mean.t()) / stddev.t();
            float prediction = svm->predict(scaled_sample);

            // --- 5. Compare ---
            if ((int)prediction == true_label) {
                correct++;
            }
            total++;

            std::cout << "Sample " << total << " | Predicted: " << (prediction == 1 ? "R" : "N") 
                      << " | Actual: " << (true_label == 1 ? "R" : "N") 
                      << (prediction == true_label ? " [OK]" : " [WRONG]") << std::endl;
        }

        // Final Score
        if (total == 0) throw std::runtime_error("No samples were processed from the CSV.");
        
        float accuracy = (float)correct / total * 100.0f;
        std::cout << "------------------------------------------" << std::endl;
        std::cout << "TOTAL ACCURACY: " << accuracy << "% (" << correct << "/" << total << ")" << std::endl;
        std::cout << "------------------------------------------\n" << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "ERROR: " << e.what() << std::endl;
        return 1;
    }
    return 0;
}