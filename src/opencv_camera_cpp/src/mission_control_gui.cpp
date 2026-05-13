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
        move2_btn_   = new QPushButton("Move to 2nd View");
        move3_btn_   = new QPushButton("Complete Mission");
        exit_btn_   = new QPushButton("Exit");
        locate_btn_ = new QPushButton("Locate Bottle");
        detect_btn_ = new QPushButton("Detect Bottle");
        home_btn_   = new QPushButton("Return Home");


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
        move2_btn_->setStyleSheet(btn_style);
        move3_btn_->setStyleSheet(btn_style);
        exit_btn_->setStyleSheet(btn_style);
        locate_btn_->setStyleSheet(btn_style);
        detect_btn_->setStyleSheet(btn_style);
        home_btn_->setStyleSheet(btn_style);
        // ---------------- LAYOUT ----------------
        QVBoxLayout *main_layout = new QVBoxLayout();
        QHBoxLayout *button_layout = new QHBoxLayout();

        button_layout->addWidget(start_btn_);
        button_layout->addWidget(move2_btn_);
        button_layout->addWidget(move3_btn_);
        button_layout->addWidget(locate_btn_);
        button_layout->addWidget(detect_btn_);
        button_layout->addWidget(stop_btn_);
        button_layout->addWidget(resume_btn_);
        button_layout->addWidget(home_btn_);
        button_layout->addWidget(exit_btn_);

        main_layout->addStretch();
        main_layout->addLayout(button_layout);
        main_layout->addStretch();

        setLayout(main_layout);

        setWindowTitle("Mission Control GUI");
        resize(900, 300);

        // ---------------- SIGNALS ----------------
        connect(start_btn_,  &QPushButton::clicked, this, &ControlGUI::start_clicked);
        connect(stop_btn_,   &QPushButton::clicked, this, &ControlGUI::stop_clicked);
        connect(resume_btn_, &QPushButton::clicked, this, &ControlGUI::resume_clicked);
        connect(home_btn_,   &QPushButton::clicked, this, &ControlGUI::home_clicked);
        connect(move2_btn_,   &QPushButton::clicked, this, &ControlGUI::move2_clicked);
        connect(move3_btn_,   &QPushButton::clicked, this, &ControlGUI::move3_clicked);
        connect(exit_btn_,   &QPushButton::clicked, this, &ControlGUI::exit_all);
        connect(locate_btn_, &QPushButton::clicked, this, &ControlGUI::locate_clicked);
        connect(detect_btn_, &QPushButton::clicked, this, &ControlGUI::detect_clicked);
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

        RCLCPP_INFO(this->get_logger(), "Published: %d", value); // 0= Home, 1=Start, 2=Stop, 3=Resume, 4=Locate, 5=Detect, 6=Move2, 7=Move3
    }

    // ---------------- STATE ----------------
    int state_; // 0=Home, 1=Running, 2=Stopped, 3=Resumed, 4=Detecting, 5=Completed
    int previous_state_; // To store the previous state for resume functionality

    // ---------------- UI ----------------
    QPushButton *start_btn_;
    QPushButton *stop_btn_;
    QPushButton *resume_btn_;
    QPushButton *move2_btn_;
    QPushButton *move3_btn_;
    QPushButton *exit_btn_;
    QPushButton *locate_btn_;
    QPushButton *detect_btn_;
    QPushButton *home_btn_;

    // ---------------- EXIT ----------------
    void exit_all()
    {
        rclcpp::shutdown();
        QApplication::quit();
    }

    // ---------------- BUTTON LOGIC ----------------
    void start_clicked()  { state_ = 1; publish(1); update_buttons(); }
    void stop_clicked()   { state_ = 2; previous_state_ = state_; publish(2); update_buttons(); }
    void home_clicked()   { state_ = 0; publish(0); update_buttons(); }
    void resume_clicked() { state_ = previous_state_; publish(3); update_buttons(); } //FIX THIS to whatever it just was 
    void move2_clicked()   { state_ = 4; publish(6); update_buttons(); }
    void move3_clicked()   { state_ = 0; publish(7); update_buttons(); }
    void locate_clicked() { state_ = 3; publish(4); update_buttons(); }
    void detect_clicked() { state_ = 5; publish(5); update_buttons(); }
    void exit_clicked()   { exit_all(); }

    // ---------------- UI STATE ----------------
    void update_buttons()
    {
        start_btn_->setVisible(state_ == 0);

        locate_btn_->setVisible(state_ == 1);

        resume_btn_->setVisible(state_ == 2);
        home_btn_->setVisible(state_ == 2);

        move2_btn_->setVisible(state_ == 3);

        detect_btn_->setVisible(state_ == 4);

        move3_btn_->setVisible(state_ == 5);

        //Show stop when not view 2 
        stop_btn_->setVisible(state_ != 2 && state_ != 0);
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