#ifndef DEFINE
#define DEFINE

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "dynamixel_sdk/dynamixel_sdk.h"
#include <cstdlib>
#include <cmath>

//_______________________________________
//
//          Model(ARM) values
//______________________________________
//
//     l       L         L       l      
//   <---> <-------> <-------> <--->
// 
//  {0}__{1}_______{2}_______{3}__{4}      
//   |            elbow            |     
//   d                             d      
//   |                             |       
//  ---                           --- 
//  base                          E.E.
//_______________________________________

#define l 0.025 // [m] 
#define L 0.130 // [m]
#define d 0.065 // [m]

// System values
#define DEVICENAME "/dev/ttyUSB0"
#define loop_hz 100


// Dynamixel setting (Custom)
#define BAUDRATE 115200
#define BASE_MOTOR_B 0 
#define BASE_MOTOR_A 1
#define XC330_DXL_ID 2
#define EE_MOTOR_A 3   
#define EE_MOTOR_B 4 
  

// Common Dynamixel values
#define PPR 4096
#define PROTOCOL_VERSION 2.0
#define ADDR_TORQUE_ENABLE 64  
#define ADDR_GOAL_POSITION 116  
#define ADDR_PRESENT_POSITION 132  
#define ADDR_OPERATING_MODE 11  
#define TORQUE_ENABLE 1
#define TORQUE_DISABLE 0


// control values
dynamixel::PortHandler *portHandler;
dynamixel::PacketHandler *packetHandler;

double base_B_ref_pos = 0;
double base_A_ref_pos = 0;
double elbow_ref_pos = 0;
double ee_A_ref_pos = 0;
double ee_B_ref_pos = 0;

double base_B_now_pos = 0;
double base_A_now_pos = 0;
double elbow_now_pos = 0;
double ee_A_now_pos = 0;
double ee_B_now_pos = 0;

int BASEisOpend = 0;
int EEisOpend = 1;

std::vector<std::pair<double, double>> points = 
    {
        {0.17, 0.0}, {0.17, 0.1}, {0.22, 0.12}, {0.22, 0.0}, {0.22, -0.12}, {0.17, -0.1}, {0.17, 0.0}
    };

double x_ee=points[0].first;
double y_ee=points[0].second;

size_t target_index = 1;
double x_target = 0;
double y_target = 0;
double step_size = 0.005;
double distance = 0;

#endif // DEFINE
