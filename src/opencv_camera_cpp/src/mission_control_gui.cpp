#include <QApplication>
#include <QWidget>
#include <QPushButton>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QPixmap>
#include <QProcess>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <thread>

class ControlGUI : public QWidget, public rclcpp::Node
{
public:
    ControlGUI()
        : QWidget(), Node("mission_control_gui"), state_(0)
    {
        publisher_ = this->create_publisher<std_msgs::msg::Int32>("system_command", 10);

        // ---------------- BUTTONS ----------------
        activate_camera_btn_ = new QPushButton("Activate Camera");
        start_btn_ = new QPushButton("Start");
        stop_btn_ = new QPushButton("Stop");
        resume_btn_ = new QPushButton("Resume");
        home_btn_ = new QPushButton("Home");
        exit_btn_ = new QPushButton("Exit");

        // ---------------- IMAGE AREA ----------------
        image_label_ = new QLabel();

        // ✅ FIXED IMAGE LOADING (ROS SAFE PATH)
        QPixmap placeholder;

        try
        {
            std::string path =
                ament_index_cpp::get_package_share_directory("opencv_camera_cpp")
                + "/resources/no_image.jpg";

            placeholder = QPixmap(QString::fromStdString(path));
        }
        catch (...)
        {
            placeholder = QPixmap();
        }

        if (placeholder.isNull())
        {
            placeholder = QPixmap(800, 500);
            placeholder.fill(Qt::black);
        }

        image_label_->setPixmap(
            placeholder.scaled(800, 500,
                               Qt::KeepAspectRatio,
                               Qt::SmoothTransformation)
        );

        image_label_->setAlignment(Qt::AlignCenter);

        // ---------------- LAYOUT ----------------
        QVBoxLayout *main_layout = new QVBoxLayout();
        QHBoxLayout *button_layout = new QHBoxLayout();

        button_layout->addWidget(activate_camera_btn_);
        button_layout->addWidget(start_btn_);
        button_layout->addWidget(stop_btn_);
        button_layout->addWidget(resume_btn_);
        button_layout->addWidget(home_btn_);
        button_layout->addWidget(exit_btn_);

        main_layout->addWidget(image_label_);
        main_layout->addLayout(button_layout);

        setLayout(main_layout);
        setWindowTitle("Mission Control GUI");

        // ---------------- SIGNALS ----------------
        connect(activate_camera_btn_, &QPushButton::clicked,
                this, &ControlGUI::activate_camera);

        connect(start_btn_, &QPushButton::clicked,
                this, &ControlGUI::start_clicked);

        connect(stop_btn_, &QPushButton::clicked,
                this, &ControlGUI::stop_clicked);

        connect(resume_btn_, &QPushButton::clicked,
                this, &ControlGUI::resume_clicked);

        connect(home_btn_, &QPushButton::clicked,
                this, &ControlGUI::home_clicked);

        connect(exit_btn_, &QPushButton::clicked,
                this, &ControlGUI::exit_all);

        update_buttons();
    }

private:
    // ---------------- ROS ----------------
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;

    void publish(int value)
    {
        auto msg = std_msgs::msg::Int32();
        msg.data = value;
        publisher_->publish(msg);

        RCLCPP_INFO(this->get_logger(), "Published: %d", value);
    }

    // ---------------- STATE ----------------
    int state_; // 0=Home, 1=Running, 2=Stopped

    // ---------------- PROCESSES ----------------
    QProcess *camera_process_ = nullptr;
    QProcess *realsense_process_ = nullptr;
    //QProcess *classifier_process_ = nullptr;

    // ---------------- UI ----------------
    QLabel *image_label_;

    QPushButton *activate_camera_btn_;
    QPushButton *start_btn_;
    QPushButton *stop_btn_;
    QPushButton *resume_btn_;
    QPushButton *home_btn_;
    QPushButton *exit_btn_;

    // ---------------- CAMERA START ----------------
    void activate_camera()
    {
        if (camera_process_ &&
            camera_process_->state() != QProcess::NotRunning)
        {
            RCLCPP_WARN(this->get_logger(), "System already running");
            return;
        }

        if (camera_process_) delete camera_process_;
        if (realsense_process_) delete realsense_process_;
        //if (classifier_process_) delete classifier_process_;

        camera_process_ = new QProcess(this);
        realsense_process_ = new QProcess(this);
        //classifier_process_ = new QProcess(this);

        camera_process_->start("/bin/bash", QStringList()
            << "-c"
            << "source /opt/ros/humble/setup.bash && "
               "ros2 run opencv_camera_cpp camera_node");

        realsense_process_->start("/bin/bash", QStringList()
            << "-c"
            << "source /opt/ros/humble/setup.bash && "
               "ros2 launch realsense2_camera rs_launch.py");

        // classifier_process_->start("/bin/bash", QStringList()
        //     << "-c"
        //     << "source /opt/ros/humble/setup.bash && "
        //        "ros2 run rs2_image_processing_package bottle_classifier_node");

        RCLCPP_INFO(this->get_logger(),
                    "Camera + Realsense + Classifier started");
    }

    // ---------------- EXIT SAFE ----------------
    void exit_all()
    {
        RCLCPP_INFO(this->get_logger(), "Shutting down Mission Control...");

        rclcpp::shutdown();

        auto stop_process = [](QProcess *&p)
        {
            if (p)
            {
                p->terminate();
                p->waitForFinished(1000);
                p->kill();
                delete p;
                p = nullptr;
            }
        };

        stop_process(camera_process_);
        stop_process(realsense_process_);
        //stop_process(classifier_process_);

        system("pkill -f camera_node");
        system("pkill -f realsense2_camera");
        system("pkill -f rs_launch");
        //system("pkill -f bottle_classifier_node");

        this->close();
    }

    // ---------------- BUTTON LOGIC ----------------
    void start_clicked()   { state_ = 1; publish(1); update_buttons(); }
    void stop_clicked()    { state_ = 2; publish(2); update_buttons(); }
    void resume_clicked()  { state_ = 1; publish(3); update_buttons(); }
    void home_clicked()    { state_ = 0; publish(0); update_buttons(); }

    // ---------------- UI STATE ----------------
    void update_buttons()
    {
        activate_camera_btn_->setVisible(state_ == 0);
        start_btn_->setVisible(state_ == 0);

        stop_btn_->setVisible(state_ == 1);
        resume_btn_->setVisible(state_ == 2);
        home_btn_->setVisible(state_ == 2);

        exit_btn_->setVisible(true);
    }
};


// ---------------- MAIN ----------------
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    QApplication app(argc, argv);

    auto gui = std::make_shared<ControlGUI>();
    gui->show();

    std::thread ros_thread([&]() {
        rclcpp::spin(gui);
    });

    int result = app.exec();

    rclcpp::shutdown();
    ros_thread.join();

    return result;
}