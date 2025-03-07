#include "crawling_robot_controller/define.h"
#include "crawling_robot_controller/dynamixel_funtion.h"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int8_multi_array.hpp>
#include <std_msgs/msg/int8.hpp>
#include <cmath>

int g_keyboard_command =0;
bool move_forward = false;
bool move_backward = false;

size_t target_index = 1;
double x_ee = points[0].first;
double y_ee = points[0].second;
rclcpp::Publisher<std_msgs::msg::Int8MultiArray>::SharedPtr gripper_state_pub;
// rclccp::Subscription<std_msgs::msg::Int8>::SharedPtr keyboard_sub; //auto 대신 쓸 수 있는 문구



void keyboard_callback(const std_msgs::msg::Int8::SharedPtr msg)
{
    if (msg->data == 3) {
        if (!move_forward) { 
            move_backward = false;
            move_forward = true;
            x_ee = points[0].first;
            y_ee = points[0].second;
        } else {

            move_forward = false;
        }
    }

    else if (msg->data == 4) {
        if (!move_backward) { 
            move_forward = false;
            move_backward = true;
            x_ee = points2[0].first;
            y_ee = points2[0].second;
        } else {
            move_backward = false;
        }
    }
}

void IK_2dim(double x, double y)
{
    double theta2 = -acos((x * x + y * y - 2 * L * L) / (2 * L * L));
    double theta1 = atan2(y, x) + asin(L * sin(-theta2) / sqrt(x * x + y * y));
    double theta3 = -(theta1 + theta2);

    base_B_ref_pos = M_PI/2;
    base_A_ref_pos = M_PI + theta1;
    elbow_ref_pos = M_PI + theta2;
    ee_A_ref_pos = M_PI + theta3;
    ee_B_ref_pos = M_PI/2;
}

void path()
{
    static bool delay_active = true;
    static int delay_counter = 0;
    const int delay_duration = 50;
    static int previous_index = -1;

    if (delay_active) 
    {
        delay_counter++;
        if (delay_counter >= delay_duration) 
        {
            delay_active = false;
            delay_counter = 0;
        }
        return;
    }

    if (target_index < points.size()) 
    {   
        if(move_forward)
        {
            x_target = points[target_index].first;
            y_target = points[target_index].second;
        }

        else if(move_backward)
        {
            x_target = points2[target_index].first;
            y_target = points2[target_index].second;
        }

        distance = sqrt(pow(x_target - x_ee, 2) + pow(y_target - y_ee, 2));

        if (distance > step_size) 
        {
            x_ee += step_size * (x_target - x_ee) / distance;
            y_ee += step_size * (y_target - y_ee) / distance;
        } 
        else 
        {
            target_index = (target_index + 1) % points.size();
        }

        if ((previous_index == 3 && target_index == 4) || (previous_index == 6 && target_index == 0)) delay_active = true;
        BASEisOpend = target_index >= 4;
        EEisOpend = target_index < 4;

        
        std_msgs::msg::Int8MultiArray gripper_state_msg;
        gripper_state_msg.data.resize(2);
        gripper_state_msg.data[0] = BASEisOpend ? 1 : 0;
        gripper_state_msg.data[1] = EEisOpend ? 1 : 0;
        gripper_state_pub->publish(gripper_state_msg);

        previous_index = target_index;
    }
}

int main(int argc, char **argv)
{
    system(("sudo chmod 666 " + std::string(DEVICENAME)).c_str());

    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("position_control");
    rclcpp::WallRate loop_rate(loop_hz);
    rclcpp::on_shutdown([]() { KILL_dynamixel(); });

    gripper_state_pub = node->create_publisher<std_msgs::msg::Int8MultiArray>("gripper_state", 10);

    auto keyboard_sub = node->create_subscription<std_msgs::msg::Int8>("keyboard_topic", 10, keyboard_callback);

    CONNECT_dynamixel();
    SET_dynamixel();
    
    while (rclcpp::ok()) 
    {
        rclcpp::spin_some(node);
        if (move_forward||move_backward)
        {
            path();
            IK_2dim(x_ee, y_ee);
            BASE_position_control(base_B_ref_pos, base_A_ref_pos);
            ELBOW_position_control(elbow_ref_pos);
            EE_position_control(ee_A_ref_pos, ee_B_ref_pos);
            RCLCPP_INFO(rclcpp::get_logger("position_control"), "ID0 : %f | ID1 : %f | ID2 : %f | ID3 : %f | ID4 : %f | Grip : Base Open: %s, EE Open: %s", 
            base_B_now_pos * 180 / M_PI,
            base_A_now_pos * 180 / M_PI,
            elbow_now_pos * 180 / M_PI,
            ee_A_now_pos * 180 / M_PI,
            ee_B_now_pos * 180 / M_PI,
            BASEisOpend ? "True" : "False",
            EEisOpend ? "True" : "False");
        }

        else
        {
            RCLCPP_INFO(rclcpp::get_logger("position_control"),"waiting for command");
        }

        loop_rate.sleep();
    }

    KILL_dynamixel(); 
    return 0;
}

