#include <algorithm>
#include <cmath>
#include <string>
#include <unordered_map>
#include <vector>
#include <SCServo_Linux/SCServo.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"

using namespace std;

class PanTiltCtrlNode : public rclcpp::Node
{
public:
    PanTiltCtrlNode() : Node("pan_tilt_ctrl_node") {
        speed_        = declare_parameter<int>("speed", 4500); // steps per second
        acceleration_ = declare_parameter<int>("acceleration", 255); // 0 - 255
        stop_button_  = declare_parameter<int>("stop_button", 4); // defaults to SixAxis/SteamDeck:L1
        min_pos_      = declare_parameter<vector<long int>>("min_pos", {1600, 1600}); // per-joint servo step limits
        max_pos_      = declare_parameter<vector<long int>>("max_pos", {2816, 2816});
        joint_ids_    = declare_parameter<vector<long int>>("joint_ids", {1, 2});
        joint_names_  = declare_parameter<vector<string>>("joint_names", {"pan_joint", "tilt_joint"}); // same order as joint_ids
        command_timeout_ = declare_parameter<double>("command_timeout", 0.5); // s without commands before holding position
        last_cmd_time_ = now();

        auto usb_port  = declare_parameter<string>("usb_port", "/dev/ttyACM0");
        auto baud_rate = declare_parameter<int>("baud_rate", 1000000); //115200 for sms, 1000000 for sts

        if (joint_ids_.size() != 2) {
            RCLCPP_ERROR(this->get_logger(), "Invalid parameter - only 2 joint IDs are supported");
            return;
        }

        if (min_pos_.size() != joint_ids_.size() || max_pos_.size() != joint_ids_.size()) {
            RCLCPP_ERROR(this->get_logger(), "Invalid parameter - min_pos/max_pos must have one entry per joint");
            return;
        }

        if (joint_names_.size() != joint_ids_.size()) {
            RCLCPP_ERROR(this->get_logger(), "Invalid parameter - joint_names must have one entry per joint ID");
            return;
        }
        for (size_t i = 0; i < joint_names_.size(); ++i) {
            joint_index_[joint_names_[i]] = i;
        }

        if(!st3215_.begin(baud_rate, usb_port.c_str())) {
            stop_ = true;
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize motors");
            return;
        }

        for (size_t i = 0; i < joint_ids_.size(); ++i) {
            stop_ = false;
            st3215_.Mode(joint_ids_[i], 0); // set to mode 0 - closed loop servo mode
        }

        joy_subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>("/joy", 10, bind(&PanTiltCtrlNode::JoyCallback, this, placeholders::_1));
        joint_cmd_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>("/joint_commands", 10, bind(&PanTiltCtrlNode::JointCmdCallback, this, placeholders::_1));
        joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
        diagnostics_publisher_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);
        timer_ = this->create_wall_timer(chrono::milliseconds(20), bind(&PanTiltCtrlNode::TimerCallback, this));

        RCLCPP_INFO(this->get_logger(), "Initialized");
    }

    ~PanTiltCtrlNode() {
        for (size_t i = 0; i < joint_ids_.size(); ++i) {
            stop_ = true;
            st3215_.EnableTorque(static_cast<uint8_t>(joint_ids_[i]), 0);
        }
        st3215_.end();
        RCLCPP_INFO(this->get_logger(), "Motors Disabled");
    }

private:
    SMS_STS st3215_;

    vector<long int> joint_ids_, min_pos_, max_pos_;
    vector<string> joint_names_;
    unordered_map<string, size_t> joint_index_; // joint name -> index into joint_ids_/limits
    int speed_, acceleration_, stop_button_;
    double command_timeout_;
    bool stop_{true};
    bool torque_enabled_{true};
    rclcpp::Time last_cmd_time_;

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_cmd_subscriber_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    sensor_msgs::msg::Joy::SharedPtr joy_msg_;
    sensor_msgs::msg::JointState::SharedPtr joint_cmd_msg_;


    void JoyCallback(const sensor_msgs::msg::Joy::SharedPtr msg) {
        if (stop_button_ < 0 || static_cast<size_t>(stop_button_) >= msg->buttons.size()) {
            RCLCPP_WARN(this->get_logger(), "Stop button index %d out of range", stop_button_);
            joy_msg_ = msg;
            return;
        }
        bool prev_pressed = joy_msg_ && static_cast<size_t>(stop_button_) < joy_msg_->buttons.size() && joy_msg_->buttons[stop_button_];
        if (msg->buttons[stop_button_] && !prev_pressed) { stop_ = !(stop_); }
        joy_msg_ = msg;
    }

    void JointCmdCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        joint_cmd_msg_ = msg;
        last_cmd_time_ = now();
    }

    void SetTorque(bool enable) {
        if (torque_enabled_ == enable) { return; }
        for (const auto id : joint_ids_) {
            st3215_.EnableTorque(static_cast<uint8_t>(id), enable ? 1 : 0);
        }
        torque_enabled_ = enable;
    }

    // Servo steps for every configured joint from the latest command, matched by
    // joint name; false if there is no fresh, complete, finite command.
    bool CommandSteps(vector<int> &steps) {
        if (!joint_cmd_msg_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "No joint command received");
            return false;
        }
        if ((now() - last_cmd_time_).seconds() > command_timeout_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Joint commands timed out - holding position");
            return false;
        }
        if (joint_cmd_msg_->position.size() < joint_cmd_msg_->name.size()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Invalid joint command received");
            return false;
        }
        steps.assign(joint_ids_.size(), 0);
        vector<bool> seen(joint_ids_.size(), false);
        for (size_t k = 0; k < joint_cmd_msg_->name.size(); ++k) {
            const auto it = joint_index_.find(joint_cmd_msg_->name[k]);
            if (it == joint_index_.end()) { continue; }
            const double position = joint_cmd_msg_->position[k];
            if (!std::isfinite(position)) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Non-finite joint command received");
                return false;
            }
            const size_t i = it->second;
            const int step = static_cast<int>(round((M_PI - position) * 4096.0 / (2 * M_PI)));
            steps[i] = clamp(step, static_cast<int>(min_pos_[i]), static_cast<int>(max_pos_[i]));
            seen[i] = true;
        }
        if (find(seen.begin(), seen.end(), false) != seen.end()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Joint command must name every configured joint");
            return false;
        }
        return true;
    }

    void TimerCallback() {
        if (stop_) {
            // Torque off whenever stopped, whether or not a valid command exists.
            SetTorque(false);
        } else {
            vector<int> steps;
            if (CommandSteps(steps)) {
                SetTorque(true);
                for (size_t i = 0; i < joint_ids_.size(); ++i) {
                    st3215_.RegWritePosEx(static_cast<uint8_t>(joint_ids_[i]), steps[i], speed_, acceleration_);
                    st3215_.RegWriteAction();
                }
            }
        }
        PublishFeedback();
    }

    // Publish measured joint states and diagnostics; joints whose feedback read
    // fails are reported in diagnostics and left out of the joint state.
    void PublishFeedback() {
        auto joint_state_msg = std::make_shared<sensor_msgs::msg::JointState>();
        joint_state_msg->header.stamp = now();

        auto diagnostic_msg = std::make_shared<diagnostic_msgs::msg::DiagnosticArray>();
        diagnostic_msg->header.stamp = now();

        for (size_t i = 0; i < joint_ids_.size(); ++i) {
            const auto &j = joint_names_[i];
            const auto u8_id = static_cast<uint8_t>(joint_ids_[i]);

            diagnostic_msgs::msg::DiagnosticStatus status;
            status.name = j;
            status.hardware_id = to_string(u8_id);

            if (st3215_.FeedBack(u8_id) == -1) {
                status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
                status.message = "no servo feedback";
                diagnostic_msg->status.push_back(status);
                continue;
            }

            const double position = M_PI - st3215_.ReadPos(u8_id)*2*M_PI/4096.0; // 1 step=2*PI/4096.0 rad
            const double speed = -1 * st3215_.ReadSpeed(u8_id)*2*M_PI/4096.0;  // 1 steps/s=2*PI/4096.0 rads/sec
            const double pwm = -1 * st3215_.ReadLoad(u8_id)/10.0; // 0-1000 : 0-100%
            const int move = st3215_.ReadMove(u8_id);
            const double temperature = st3215_.ReadTemper(u8_id); // 1 : 1 degree Celcius
            const double voltage = st3215_.ReadVoltage(u8_id)/10.0; // 1 : 0.1 V
            const double current = st3215_.ReadCurrent(u8_id)*6.5/1000; // 1 : 6.5/1000 A

            joint_state_msg->name.push_back(j);
            joint_state_msg->position.push_back(position);
            joint_state_msg->velocity.push_back(speed);
            joint_state_msg->effort.push_back(current);

            status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
            status.message = stop_ ? "stopped (torque off)" : j;
            for (const auto &kv : vector<pair<string, string>>{
                     {"pwm", to_string(pwm)}, {"move", to_string(move)},
                     {"temperature", to_string(temperature)}, {"voltage", to_string(voltage)},
                     {"current", to_string(current)}}) {
                diagnostic_msgs::msg::KeyValue value;
                value.key = kv.first;
                value.value = kv.second;
                status.values.push_back(value);
            }
            diagnostic_msg->status.push_back(status);
        }

        if (!joint_state_msg->name.empty()) {
            joint_state_publisher_->publish(*joint_state_msg);
        }
        diagnostics_publisher_->publish(*diagnostic_msg);
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = make_shared<PanTiltCtrlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
