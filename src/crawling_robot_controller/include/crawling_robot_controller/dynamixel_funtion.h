#ifndef DYNAMIXEL_FUNTION
#define DYNAMIXEL_FUNTION

#include "dynamixel_sdk/dynamixel_sdk.h"
#include "define.h"

dynamixel::GroupSyncWrite *groupSyncWrite;
dynamixel::GroupSyncRead *groupSyncRead;

void CONNECT_dynamixel() 
{
    portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);  
    portHandler->openPort();
    portHandler->setBaudRate(BAUDRATE);
}

void SET_dynamixel() 
{
    packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    groupSyncWrite = new dynamixel::GroupSyncWrite(portHandler, packetHandler, ADDR_GOAL_POSITION, 4);
    groupSyncRead = new dynamixel::GroupSyncRead(portHandler, packetHandler, ADDR_PRESENT_POSITION, 4);

    packetHandler->write1ByteTxRx(portHandler, BASE_MOTOR_B, ADDR_OPERATING_MODE, 3, nullptr);
    packetHandler->write1ByteTxRx(portHandler, BASE_MOTOR_B, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, nullptr);
    packetHandler->write1ByteTxRx(portHandler, BASE_MOTOR_A, ADDR_OPERATING_MODE, 3, nullptr);
    packetHandler->write1ByteTxRx(portHandler, BASE_MOTOR_A, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, nullptr);

    packetHandler->write1ByteTxRx(portHandler, XC330_DXL_ID, ADDR_OPERATING_MODE, 3, nullptr);
    packetHandler->write1ByteTxRx(portHandler, XC330_DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, nullptr);
    
    packetHandler->write1ByteTxRx(portHandler, EE_MOTOR_A, ADDR_OPERATING_MODE, 3, nullptr);
    packetHandler->write1ByteTxRx(portHandler, EE_MOTOR_A, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, nullptr);
    packetHandler->write1ByteTxRx(portHandler, EE_MOTOR_B, ADDR_OPERATING_MODE, 3, nullptr);
    packetHandler->write1ByteTxRx(portHandler, EE_MOTOR_B, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, nullptr);
}

void KILL_dynamixel() 
{
    packetHandler->write1ByteTxRx(portHandler, BASE_MOTOR_B, ADDR_TORQUE_ENABLE, TORQUE_DISABLE, nullptr);
    packetHandler->write1ByteTxRx(portHandler, BASE_MOTOR_A, ADDR_TORQUE_ENABLE, TORQUE_DISABLE, nullptr);

    packetHandler->write1ByteTxRx(portHandler, XC330_DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE, nullptr);
    
    packetHandler->write1ByteTxRx(portHandler, EE_MOTOR_A, ADDR_TORQUE_ENABLE, TORQUE_DISABLE, nullptr);
    packetHandler->write1ByteTxRx(portHandler, EE_MOTOR_B, ADDR_TORQUE_ENABLE, TORQUE_DISABLE, nullptr);

    portHandler->closePort();
    delete groupSyncWrite;
    delete groupSyncRead;
}

void BASE_position_control(double base_B_ref_pos, double base_A_ref_pos) 
{
    uint32_t base_B_ref_pos_step = static_cast<uint32_t>(base_B_ref_pos * PPR / (2 * M_PI));
    uint8_t base_B_param_goal_position[4] = {
        DXL_LOBYTE(DXL_LOWORD(base_B_ref_pos_step)),
        DXL_HIBYTE(DXL_LOWORD(base_B_ref_pos_step)),
        DXL_LOBYTE(DXL_HIWORD(base_B_ref_pos_step)),
        DXL_HIBYTE(DXL_HIWORD(base_B_ref_pos_step))
    };
    groupSyncWrite->addParam(BASE_MOTOR_B, base_B_param_goal_position);

    uint32_t base_A_ref_pos_step = static_cast<uint32_t>(base_A_ref_pos * PPR / (2 * M_PI));
    uint8_t base_A_param_goal_position[4] = {
        DXL_LOBYTE(DXL_LOWORD(base_A_ref_pos_step)),
        DXL_HIBYTE(DXL_LOWORD(base_A_ref_pos_step)),
        DXL_LOBYTE(DXL_HIWORD(base_A_ref_pos_step)),
        DXL_HIBYTE(DXL_HIWORD(base_A_ref_pos_step))
    };
    groupSyncWrite->addParam(BASE_MOTOR_A, base_A_param_goal_position);

    groupSyncWrite->txPacket();
    groupSyncWrite->clearParam();

    groupSyncRead->clearParam();
    groupSyncRead->addParam(BASE_MOTOR_B);
    groupSyncRead->addParam(BASE_MOTOR_A);
    groupSyncRead->txRxPacket();

    uint32_t base_B_now_pos_step = groupSyncRead->getData(BASE_MOTOR_B, ADDR_PRESENT_POSITION, 4);
    uint32_t base_A_now_pos_step = groupSyncRead->getData(BASE_MOTOR_A, ADDR_PRESENT_POSITION, 4);

    base_B_now_pos = static_cast<double>(base_B_now_pos_step) * (2 * M_PI) / PPR;
    base_A_now_pos = static_cast<double>(base_A_now_pos_step) * (2 * M_PI) / PPR;
}

void ELBOW_position_control(double elbow_ref_pos) 
{
    uint32_t elbow_ref_pos_step = static_cast<uint32_t>(elbow_ref_pos * PPR / (2 * M_PI));
    uint8_t elbow_param_goal_position[4] = {
        DXL_LOBYTE(DXL_LOWORD(elbow_ref_pos_step)),
        DXL_HIBYTE(DXL_LOWORD(elbow_ref_pos_step)),
        DXL_LOBYTE(DXL_HIWORD(elbow_ref_pos_step)),
        DXL_HIBYTE(DXL_HIWORD(elbow_ref_pos_step))
    };
    groupSyncWrite->addParam(XC330_DXL_ID, elbow_param_goal_position);
    
    groupSyncWrite->txPacket();
    groupSyncWrite->clearParam();

    groupSyncRead->clearParam();
    groupSyncRead->addParam(XC330_DXL_ID);
    groupSyncRead->txRxPacket();

    uint32_t elbow_now_pos_step = groupSyncRead->getData(XC330_DXL_ID, ADDR_PRESENT_POSITION, 4);
    elbow_now_pos = static_cast<double>(elbow_now_pos_step) * (2 * M_PI) / PPR;
}

void EE_position_control(double ee_A_ref_pos, double ee_B_ref_pos) 
{
    uint32_t ee_A_ref_pos_step = static_cast<uint32_t>(ee_A_ref_pos * PPR / (2 * M_PI));
    uint8_t ee_A_param_goal_position[4] = {
        DXL_LOBYTE(DXL_LOWORD(ee_A_ref_pos_step)),
        DXL_HIBYTE(DXL_LOWORD(ee_A_ref_pos_step)),
        DXL_LOBYTE(DXL_HIWORD(ee_A_ref_pos_step)),
        DXL_HIBYTE(DXL_HIWORD(ee_A_ref_pos_step))
    };
    groupSyncWrite->addParam(EE_MOTOR_A, ee_A_param_goal_position);

    uint32_t ee_B_ref_pos_step = static_cast<uint32_t>(ee_B_ref_pos * PPR / (2 * M_PI));
    uint8_t ee_B_param_goal_position[4] = {
        DXL_LOBYTE(DXL_LOWORD(ee_B_ref_pos_step)),
        DXL_HIBYTE(DXL_LOWORD(ee_B_ref_pos_step)),
        DXL_LOBYTE(DXL_HIWORD(ee_B_ref_pos_step)),
        DXL_HIBYTE(DXL_HIWORD(ee_B_ref_pos_step))
    };
    groupSyncWrite->addParam(EE_MOTOR_B, ee_B_param_goal_position);

    groupSyncWrite->txPacket();
    groupSyncWrite->clearParam();

    groupSyncRead->clearParam();
    groupSyncRead->addParam(EE_MOTOR_A);
    groupSyncRead->addParam(EE_MOTOR_B);
    groupSyncRead->txRxPacket();

    uint32_t ee_A_now_pos_step = groupSyncRead->getData(EE_MOTOR_A, ADDR_PRESENT_POSITION, 4);
    uint32_t ee_B_now_pos_step = groupSyncRead->getData(EE_MOTOR_B, ADDR_PRESENT_POSITION, 4);

    ee_A_now_pos = static_cast<double>(ee_A_now_pos_step) * (2 * M_PI) / PPR;
    ee_B_now_pos = static_cast<double>(ee_B_now_pos_step) * (2 * M_PI) / PPR;
}

#endif // DYNAMIXEL_FUNTION
