#include "crawling_robot_controller/define.h"
#include "crawling_robot_controller/dynamixel_funtion.h"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int8_multi_array.hpp>
#include <std_msgs/msg/int8.hpp>
#include <cmath>

int g_keyboard_command =0;
int target_index = 1;
bool free_mode = false;
double move_size = 0.000;
double length_size = 0.000;
rclcpp::Publisher<std_msgs::msg::Int8MultiArray>::SharedPtr gripper_state_pub;

std::vector<std::pair<double, double>> getPoints(double move_size, double length_size) {
    return {
        {0.185 + move_size, 0.0},
        {0.185 + move_size, 0.05 + length_size},
        {0.22 + move_size, 0.12 + length_size},
        {0.22 + move_size, 0.0},
        {0.22 + move_size, -0.12 - length_size},
        {0.185 + move_size, -0.05 - length_size},
        {0.185, 0.0}
    };
}
auto points = getPoints(move_size,length_size);
double x_ee = points[0].first;
double y_ee = points[0].second;

void keyboard_callback(const std_msgs::msg::Int8::SharedPtr msg)
{
    if (free_mode){
        if (msg->data == 0) {
            move_forward = false;
            move_backward = false;
            RCLCPP_INFO(rclcpp::get_logger("position_control"), "free_mode_stop");
        } 
        else if (msg->data == 3) {
            move_forward = true;
            move_backward = false;
            RCLCPP_INFO(rclcpp::get_logger("position_control"), "free_move_forward");
        }
        else if (msg->data == 4) {
            move_forward = false;
            move_backward = true;
            RCLCPP_INFO(rclcpp::get_logger("position_control"), "free_move_forward");
        }
        else if (msg->data == 7) {
            move_forward = false;
            move_backward = false;
            free_mode = false;
            RCLCPP_INFO(rclcpp::get_logger("position_control"), "free_move_out");
        }
    }
    else if (free_mode == false){
        if (msg->data == 0) {
            move_forward = false;
            move_backward = false;
            // RCLCPP_INFO(rclcpp::get_logger("position_control"), "move_stop");
        } 
        else if (msg->data == 3) {
            move_forward = true;
            move_backward = false;
            // RCLCPP_INFO(rclcpp::get_logger("position_control"), "move_forward");
        }
        else if (msg->data == 4) {
            move_forward = false;
            move_backward = true;
            // RCLCPP_INFO(rclcpp::get_logger("position_control"), "move_forward");
        }
        else if (msg->data == 5) {
            move_size += 0.01;
            length_size -= 0.03;
            move_size = move_size > 0.03 ? 0.03 : move_size;
            length_size = length_size < -0.06 ? -0.06 : length_size;

            // if (move_size == 0.01) length_size -= 0.03;
            // else if (move_size == 0.02) length_size -= 0.05;
            // else if (move_size == 0.03) length_size -= 0.07;
            
            
            points = getPoints(move_size, length_size);
            RCLCPP_INFO(rclcpp::get_logger("position_control"), "move_size : %f", move_size);
        }
        else if (msg->data == 6) {
            move_size -= 0.01;
            length_size += 0.01;

            // if (move_size == -0.01) length_size += 0.03;
            // else if (move_size == -0.02) length_size += 0.04;
            // else if (move_size == -0.03) length_size += 0.05;
            move_size = move_size < -0.03 ? -0.03 : move_size;
            length_size = length_size > 0.06 ? 0.06 : length_size;
            
            points = getPoints(move_size, length_size);
            RCLCPP_INFO(rclcpp::get_logger("position_control"), "move_size : %f", move_size);
        }
        else if(msg->data == 7){
            free_mode = true;
            RCLCPP_INFO(rclcpp::get_logger("position_control"), "free_move_in");
        }
    }
    //RCLCPP_INFO(rclcpp::get_logger("position_control"), "keyboard_callback : %d, move_size = %f", msg->data, move_size);
}

void IK_2dim(double x, double y)
{
    double theta2 = -acos((x * x + y * y - 2 * L * L) / (2 * L * L));
    double theta1 = atan2(y, x) + asin(L * sin(-theta2) / sqrt(x * x + y * y));
    double theta3 = (theta1 + theta2);

    base_ref_pos = M_PI + theta1 - M_PI / 2;
    elbow_ref_pos = M_PI + theta2;
    ee_ref_pos = M_PI + theta3 + M_PI / 2;

    // RCLCPP_INFO(rclcpp::get_logger("position_control"), "theta1 : %f | theta2 : %f | theta3 : %f", theta1 * 180 / M_PI, theta2 * 180 / M_PI, theta3 * 180 / M_PI);
    // RCLCPP_INFO(rclcpp::get_logger("position_control"), "base_ref_pos : %f | elbow_ref_pos : %f | ee_ref_pos : %f", base_ref_pos * 180 / M_PI, elbow_ref_pos * 180 / M_PI, ee_ref_pos * 180 / M_PI);

}



void path()
{
    static bool delay_active = true;
    static int delay_counter = 0;
    const int delay_duration = 20;
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
        points = getPoints(move_size,length_size);
        x_target = points[target_index].first;
        y_target = points[target_index].second;
        distance = sqrt(pow(x_target - x_ee, 2) + pow(y_target - y_ee, 2));
        // RCLCPP_INFO(rclcpp::get_logger("position_control"), "x_target : %f | y_target : %f | x_ee : %f | y_ee : %f | distance : %f", x_target, y_target, x_ee, y_ee, distance);
        if (distance > step_size) 
        {
            x_ee += step_size * (x_target - x_ee) / distance;
            y_ee += step_size * (y_target - y_ee) / distance;
        } 
        else 
        {
            if(move_forward){
                target_index = (target_index + 1) % points.size();
            }

            else if(move_backward){
                target_index = target_index == 0 ? points.size() : target_index;
                target_index = (target_index - 1) % points.size();
            }


        std_msgs::msg::Int8MultiArray gripper_state_msg;

        if(move_forward){
        if ((previous_index == 3 && target_index == 4) || (previous_index == 6 && target_index == 0)) delay_active = true;

        BASEisOpend = target_index >= 4;
        EEisOpend = target_index < 4;
        gripper_state_msg.data.resize(2);
        gripper_state_msg.data[0] = BASEisOpend ? 1 : 0;
        gripper_state_msg.data[1] = EEisOpend ? 1 : 0;
        }

        else if(move_backward){
        //if ((previous_index == 3 && target_index == 2) || (previous_index == 0 && target_index == 6)) delay_active = true;
        
        BASEisOpend = target_index <=  4;
        EEisOpend = target_index > 4;
        gripper_state_msg.data.resize(2);
        gripper_state_msg.data[0] = BASEisOpend ? 1 : 0;
        gripper_state_msg.data[1] = EEisOpend ? 1 : 0;
        }

        gripper_state_pub->publish(gripper_state_msg);
        RCLCPP_INFO(rclcpp::get_logger("position_control"), "x_target: %f, y_target: %f, move_size: %f, length_size: %f, target_index : %d", x_target, y_target, move_size, length_size,  target_index);

        previous_index = target_index;
        }
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
    auto keyboard_sub = node->create_subscription<std_msgs::msg::Int8>("keyboard", 10, keyboard_callback);

    CONNECT_dynamixel();
    SET_dynamixel();

    while (rclcpp::ok()) 
    {
        // RCLCPP_INFO(rclcpp::get_logger("position_control"), "You can do it! ");
        if(move_forward||move_backward)
        {
        // RCLCPP_INFO(rclcpp::get_logger("position_control"), "OH yeah! ");
        path();
        IK_2dim(x_ee, y_ee);
        BASE_position_control(base_ref_pos);
        ELBOW_position_control(elbow_ref_pos);
        EE_position_control(ee_ref_pos);
        
        // RCLCPP_INFO(rclcpp::get_logger("position_control"), "ID1 : %f | ID2 : %f | ID3 : %f  | Grip : Base Open: %s, EE Open: %s", 
        // base_now_pos * 180 / M_PI,
        // elbow_now_pos * 180 / M_PI,
        // ee_now_pos * 180 / M_PI,
        // BASEisOpend ? "True" : "False",
        // EEisOpend ? "True" : "False");
        }

        rclcpp::spin_some(node);
        loop_rate.sleep();
    }

    KILL_dynamixel(); 
    return 0;
}