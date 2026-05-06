#include <QApplication>
#include <QWidget>
#include <QPushButton>
#include <QVBoxLayout>
#include <QHBoxLayout>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>

#include <thread>


class ControlGUI : public QWidget, public rclcpp::Node
{
public:
    ControlGUI()
        : QWidget(), Node("mission_control_gui"), state_(0)
    {
        publisher_ = this->create_publisher<std_msgs::msg::Int32>("system_command", 10);

        // ---------------- BUTTONS ----------------
        start_btn_  = new QPushButton("Start");
        stop_btn_   = new QPushButton("Stop");
        resume_btn_ = new QPushButton("Resume");
        home_btn_   = new QPushButton("Home");
        exit_btn_   = new QPushButton("Exit");

        QString btn_style =
            "QPushButton {"
            "font-size: 22px;"
            "padding: 15px 30px;"
            "min-width: 160px;"
            "min-height: 60px;"
            "}";

        start_btn_->setStyleSheet(btn_style);
        stop_btn_->setStyleSheet(btn_style);
        resume_btn_->setStyleSheet(btn_style);
        home_btn_->setStyleSheet(btn_style);
        exit_btn_->setStyleSheet(btn_style);

        // ---------------- LAYOUT ----------------
        QVBoxLayout *main_layout = new QVBoxLayout();
        QHBoxLayout *button_layout = new QHBoxLayout();

        button_layout->addWidget(start_btn_);
        button_layout->addWidget(stop_btn_);
        button_layout->addWidget(resume_btn_);
        button_layout->addWidget(home_btn_);
        button_layout->addWidget(exit_btn_);

        main_layout->addStretch();          // centers buttons vertically
        main_layout->addLayout(button_layout);
        main_layout->addStretch();

        setLayout(main_layout);

        setWindowTitle("Mission Control GUI");
        resize(900, 300);  // smaller, clean control panel

        // ---------------- SIGNALS ----------------
        connect(start_btn_,  &QPushButton::clicked, this, &ControlGUI::start_clicked);
        connect(stop_btn_,   &QPushButton::clicked, this, &ControlGUI::stop_clicked);
        connect(resume_btn_, &QPushButton::clicked, this, &ControlGUI::resume_clicked);
        connect(home_btn_,   &QPushButton::clicked, this, &ControlGUI::home_clicked);
        connect(exit_btn_,   &QPushButton::clicked, this, &ControlGUI::exit_all);

        update_buttons();
    }

private:

    // ---------------- ROS ----------------
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;

    void publish(int value)
    {
        std_msgs::msg::Int32 msg;
        msg.data = value;
        publisher_->publish(msg);

        RCLCPP_INFO(this->get_logger(), "Published: %d", value);
    }

    // ---------------- STATE ----------------
    int state_; // 0=Home, 1=Running, 2=Stopped

    // ---------------- UI ----------------
    QPushButton *start_btn_;
    QPushButton *stop_btn_;
    QPushButton *resume_btn_;
    QPushButton *home_btn_;
    QPushButton *exit_btn_;

    // ---------------- EXIT ----------------
    void exit_all()
    {
        rclcpp::shutdown();
        QApplication::quit();
    }

    // ---------------- BUTTON LOGIC ----------------
    void start_clicked()  { state_ = 1; publish(1); update_buttons(); }
    void stop_clicked()   { state_ = 2; publish(2); update_buttons(); }
    void resume_clicked() { state_ = 3; publish(3); update_buttons(); }
    void home_clicked()   { state_ = 0; publish(0); update_buttons(); }

    // ---------------- UI STATE ----------------
    void update_buttons()
    {
        start_btn_->setVisible(state_ == 0);

        stop_btn_->setVisible(state_ == 1);
        resume_btn_->setVisible(state_ == 3);
        home_btn_->setVisible(state_ == 0);

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