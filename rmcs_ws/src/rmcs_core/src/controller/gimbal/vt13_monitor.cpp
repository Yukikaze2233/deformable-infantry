#include <cstdint>
#include <string>
#include <string_view>

#include <eigen3/Eigen/Dense>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>

#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/VT13_switch.hpp>
#include <rmcs_msgs/keyboard.hpp>
#include <rmcs_msgs/mouse.hpp>

namespace rmcs_core::controller::gimbal {

class VT13Monitor
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    VT13Monitor()
        : rclcpp::Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {
        register_input("/remote/vt13/joystick/right", joystick_right_, false);
        register_input("/remote/vt13/joystick/left", joystick_left_, false);
        register_input("/remote/vt13/switch/state", switch_state_, false);
        register_input("/remote/vt13/mouse/velocity", mouse_velocity_, false);
        register_input("/remote/vt13/mouse/mouse_wheel", mouse_wheel_, false);
        register_input("/remote/vt13/mouse", mouse_, false);
        register_input("/remote/vt13/keyboard", keyboard_, false);
        register_input("/remote/vt13/mouse/middle", mouse_middle_, false);
        register_input("/remote/vt13/button/pause", pause_button_, false);
        register_input("/remote/vt13/button/left", button_left_, false);
        register_input("/remote/vt13/button/right", button_right_, false);
        register_input("/remote/vt13/button/trigger", trigger_button_, false);
        register_input("/remote/vt13/rotary_knob", rotary_knob_, false);
    }

    void before_updating() override {
        if (defaults_initialized_)
            return;

        connected_ = joystick_right_.ready() && joystick_left_.ready() && switch_state_.ready()
                  && mouse_velocity_.ready() && mouse_wheel_.ready() && mouse_.ready()
                  && keyboard_.ready() && mouse_middle_.ready() && pause_button_.ready()
                  && button_left_.ready() && button_right_.ready() && trigger_button_.ready()
                  && rotary_knob_.ready();

        if (!joystick_right_.ready())
            joystick_right_.make_and_bind_directly(Eigen::Vector2d::Zero());
        if (!joystick_left_.ready())
            joystick_left_.make_and_bind_directly(Eigen::Vector2d::Zero());
        if (!switch_state_.ready())
            switch_state_.make_and_bind_directly(rmcs_msgs::VT13Switch::UNKNOWN);
        if (!mouse_velocity_.ready())
            mouse_velocity_.make_and_bind_directly(Eigen::Vector2d::Zero());
        if (!mouse_wheel_.ready())
            mouse_wheel_.make_and_bind_directly(0.0);
        if (!mouse_.ready())
            mouse_.make_and_bind_directly(rmcs_msgs::Mouse::zero());
        if (!keyboard_.ready())
            keyboard_.make_and_bind_directly(rmcs_msgs::Keyboard::zero());
        if (!mouse_middle_.ready())
            mouse_middle_.make_and_bind_directly(false);
        if (!pause_button_.ready())
            pause_button_.make_and_bind_directly(false);
        if (!button_left_.ready())
            button_left_.make_and_bind_directly(false);
        if (!button_right_.ready())
            button_right_.make_and_bind_directly(false);
        if (!trigger_button_.ready())
            trigger_button_.make_and_bind_directly(false);
        if (!rotary_knob_.ready())
            rotary_knob_.make_and_bind_directly(0.0);

        defaults_initialized_ = true;
    }

    void update() override {
        if (!connected_) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "VT13 monitor is using defaults because /remote/vt13/* topics are not connected");
            return;
        }

        const auto keyboard_mask = pack_keyboard(*keyboard_);
        const auto keys = active_keys(*keyboard_);

        RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 100,
            "VT13 switch=%s jr=(%.3f, %.3f) jl=(%.3f, %.3f) mv=(%.3f, %.3f) wheel=%.3f knob=%.3f",
            switch_to_cstr(*switch_state_), joystick_right_->x(), joystick_right_->y(),
            joystick_left_->x(), joystick_left_->y(), mouse_velocity_->x(),
            mouse_velocity_->y(), *mouse_wheel_, *rotary_knob_);
        RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "VT13 buttons pause=%d left=%d right=%d trigger=%d mouse[l=%d r=%d m=%d] keyboard=0x%04X keys=%s",
            static_cast<int>(*pause_button_), static_cast<int>(*button_left_),
            static_cast<int>(*button_right_), static_cast<int>(*trigger_button_),
            static_cast<int>(mouse_->left), static_cast<int>(mouse_->right),
            static_cast<int>(*mouse_middle_), static_cast<unsigned int>(keyboard_mask),
            keys.c_str());
    }

private:
    static const char* switch_to_cstr(rmcs_msgs::VT13Switch switch_state) {
        switch (switch_state) {
        case rmcs_msgs::VT13Switch::C: return "C";
        case rmcs_msgs::VT13Switch::N: return "N";
        case rmcs_msgs::VT13Switch::S: return "S";
        case rmcs_msgs::VT13Switch::UNKNOWN: return "UNKNOWN";
        }

        return "INVALID";
    }

    static uint16_t pack_keyboard(const rmcs_msgs::Keyboard& keyboard) {
        return static_cast<uint16_t>(keyboard.w) | (static_cast<uint16_t>(keyboard.s) << 1)
             | (static_cast<uint16_t>(keyboard.a) << 2)
             | (static_cast<uint16_t>(keyboard.d) << 3)
             | (static_cast<uint16_t>(keyboard.shift) << 4)
             | (static_cast<uint16_t>(keyboard.ctrl) << 5)
             | (static_cast<uint16_t>(keyboard.q) << 6)
             | (static_cast<uint16_t>(keyboard.e) << 7)
             | (static_cast<uint16_t>(keyboard.r) << 8)
             | (static_cast<uint16_t>(keyboard.f) << 9)
             | (static_cast<uint16_t>(keyboard.g) << 10)
             | (static_cast<uint16_t>(keyboard.z) << 11)
             | (static_cast<uint16_t>(keyboard.x) << 12)
             | (static_cast<uint16_t>(keyboard.c) << 13)
             | (static_cast<uint16_t>(keyboard.v) << 14)
             | (static_cast<uint16_t>(keyboard.b) << 15);
    }

    static std::string active_keys(const rmcs_msgs::Keyboard& keyboard) {
        std::string result;

        auto append_key = [&result](bool pressed, std::string_view name) {
            if (!pressed)
                return;
            if (!result.empty())
                result += ',';
            result += name;
        };

        append_key(keyboard.w, "w");
        append_key(keyboard.s, "s");
        append_key(keyboard.a, "a");
        append_key(keyboard.d, "d");
        append_key(keyboard.shift, "shift");
        append_key(keyboard.ctrl, "ctrl");
        append_key(keyboard.q, "q");
        append_key(keyboard.e, "e");
        append_key(keyboard.r, "r");
        append_key(keyboard.f, "f");
        append_key(keyboard.g, "g");
        append_key(keyboard.z, "z");
        append_key(keyboard.x, "x");
        append_key(keyboard.c, "c");
        append_key(keyboard.v, "v");
        append_key(keyboard.b, "b");

        if (result.empty())
            return "-";
        return result;
    }

    bool defaults_initialized_{false};
    bool connected_{false};

    InputInterface<Eigen::Vector2d> joystick_right_;
    InputInterface<Eigen::Vector2d> joystick_left_;
    InputInterface<rmcs_msgs::VT13Switch> switch_state_;
    InputInterface<Eigen::Vector2d> mouse_velocity_;
    InputInterface<double> mouse_wheel_;
    InputInterface<rmcs_msgs::Mouse> mouse_;
    InputInterface<rmcs_msgs::Keyboard> keyboard_;
    InputInterface<bool> mouse_middle_;
    InputInterface<bool> pause_button_;
    InputInterface<bool> button_left_;
    InputInterface<bool> button_right_;
    InputInterface<bool> trigger_button_;
    InputInterface<double> rotary_knob_;
};

} // namespace rmcs_core::controller::gimbal

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::gimbal::VT13Monitor, rmcs_executor::Component)
