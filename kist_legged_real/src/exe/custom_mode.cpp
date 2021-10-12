/************************************************************************
Copyright (c) 2021, Korea Institude of Science and Technology(Kist). All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

#include <ros/ros.h>
#include <string>
#include <pthread.h>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <kist_legged_msgs/LowCmd.h>
#include <kist_legged_msgs/LowState.h>
#include "convert.h"

#ifdef SDK3_1
using namespace aliengo;
#endif
#ifdef SDK3_2
using namespace UNITREE_LEGGED_SDK;
#endif

#define CONTROL_PERIOD 2000 //micro second

template <typename TLCM>
void *update_loop(void *param)
{
    TLCM *data = (TLCM *)param;
    while (ros::ok)
    {
        data->Recv();
        usleep(CONTROL_PERIOD);
    }
}

double jointLinearInterpolation(double initPos, double targetPos, double rate)
{
    double p;
    rate = std::min(std::max(rate, 0.0), 1.0);
    p = initPos * (1 - rate) + targetPos * rate;
    return p;
}

template <typename TCmd, typename TState, typename TLCM>
int mainHelper(int argc, char *argv[], TLCM &roslcm)
{

    //float torque[12];
    float torque;

    std::cout << "WARNING: Control level is set to LOW-level." << std::endl
              << "Make sure the robot is hung up." << std::endl
              << "Press Enter to continue..." << std::endl;

    std::cout << "Input Torque value" << std::endl;
    std::cin >> torque;
    std::cin.ignore();
    std::cout << "Torque : " << torque << "(N/m)" << std::endl
              << "Check the Torque value" << std::endl;
    std::cin.ignore();
    /*
    for (int i = 0; i < 12; i++)
    {
    std::cout << "Input Torque value" << std::endl;
    std::cin >> torque[i];
    std::cin.ignore();
    std::cout << "Torque [" << i << "] : " << torque[i] << "(N/m)" << std::endl
              << "Check the Torque value" << std::endl;
    std::cin.ignore();
    }
  */
    ros::NodeHandle n;    
    int control_freq = 1000000/CONTROL_PERIOD;
    double control_period_s = CONTROL_PERIOD/1000000.0;
    ros::Rate loop_rate(control_freq);

    long motiontime = 0;
    //float torque = 0;
    TCmd SendLowLCM = {0};
    TState RecvLowLCM = {0};
    kist_legged_msgs::LowCmd SendLowROS;
    kist_legged_msgs::LowState RecvLowROS;

    roslcm.SubscribeState();

    pthread_t tid;
    pthread_create(&tid, NULL, update_loop<TLCM>, &roslcm);

    SendLowROS.levelFlag = LOWLEVEL;
    for (int i = 0; i < 12; i++)
    {
        SendLowROS.motorCmd[i].mode = 0x0A; // motor switch to servo (PMSM) mode
    }

    CHighLevelController HighlevelCtrl(control_period_s);

    while (ros::ok())
    {
        motiontime++;
        roslcm.Get(RecvLowLCM);
        RecvLowROS = ToRos(RecvLowLCM);
        printf("FL_1 position: %f\n", RecvLowROS.motorState[FL_1].q);
        printf("FL_1 torque: %f\n", RecvLowROS.motorState[FL_1].tauEst);

        // // gravity compensation
        // SendLowROS.motorCmd[FR_0].tau = -0.65f;
        // SendLowROS.motorCmd[FL_0].tau = +0.65f;
        // SendLowROS.motorCmd[RR_0].tau = -0.65f;
        // SendLowROS.motorCmd[RL_0].tau = +0.65f;

        // if (motiontime >= 500)
        // {
        //     //torque = (0 - RecvLowROS.motorState[FL_1].q) * 10.0f + (0 - RecvLowROS.motorState[FL_1].dq) * 1.0f;
        //     if (torque > 5.0f)
        //         torque = 5.0f;
        //     if (torque < -5.0f)
        //         torque = -5.0f;

        //     SendLowROS.motorCmd[FL_1].q = PosStopF;
        //     SendLowROS.motorCmd[FL_1].dq = VelStopF;
        //     SendLowROS.motorCmd[FL_1].Kp = 0;
        //     SendLowROS.motorCmd[FL_1].Kd = 0;
        //     SendLowROS.motorCmd[FL_1].tau = torque;
        // }

        //------------------ High Level Controller --------------------//

        CHighLevelController.read();
        
        CHighLevelController.compute_controller();

        //initialize 
        SendLowROS.motorCmd[FL_1].q = PosStopF;
        //     SendLowROS.motorCmd[FL_1].dq = VelStopF;
        //     SendLowROS.motorCmd[FL_1].Kp = 0;
        //     SendLowROS.motorCmd[FL_1].Kd = 0;
        //     SendLowROS.motorCmd[FL_1].tau = torque;

        CHighLevelController.write();

        //-------------------------------------------------------------//


        SendLowLCM = ToLcm(SendLowROS, SendLowLCM);
        roslcm.Send(SendLowLCM);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "custom_ros_mode");
    std::string firmwork;
    ros::param::get("/firmwork", firmwork);

    std::string robot_name;
    UNITREE_LEGGED_SDK::LeggedType rname;
    ros::param::get("/robot_name", robot_name);
    if (strcasecmp(robot_name.c_str(), "A1") == 0)
        rname = UNITREE_LEGGED_SDK::LeggedType::A1;
    else if (strcasecmp(robot_name.c_str(), "Aliengo") == 0)
        rname = UNITREE_LEGGED_SDK::LeggedType::Aliengo;

    // UNITREE_LEGGED_SDK::Control control(rname, UNITREE_LEGGED_SDK::LOWLEVEL);
    // UNITREE_LEGGED_SDK::InitEnvironment();
    UNITREE_LEGGED_SDK::LCM roslcm(LOWLEVEL);
    mainHelper<UNITREE_LEGGED_SDK::LowCmd, UNITREE_LEGGED_SDK::LowState, UNITREE_LEGGED_SDK::LCM>(argc, argv, roslcm);
}