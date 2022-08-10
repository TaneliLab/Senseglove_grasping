#ifndef BRIDGE_HW_SIM_H
#define BRIDGE_HW_SIM_H

// ROS include
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include "senseglove_shared_resources/FingerDistanceFloats.h"
#include <senseglove_hardware/senseglove_robot.h>
#include <senseglove_hardware_builder/hardware_builder.h>
#include <gazebo_msgs/ContactsState.h>

#include <vector>
#include <string>
#include <map>

class brige_hw_sim{

    public :
        brige_hw_sim();
        ~brige_hw_sim();
        

    private :
        ros::NodeHandle n_; // ROS NODE Handle 
        ros::Publisher pub_sim[15]; //publisher for simulation
        ros::Subscriber sub_hw; //subscriber for joint
        ros::Subscriber sub_hw_init; //subscriber for calibration
        ros::Subscriber sub_dist; //subscriber for finger distance
        ros::Subscriber sub_force[10]; //subscriber for force from gazebo
        double distance; //distance for fingers

        void jointCallback(sensor_msgs::JointState joint);
        void initJointCallback(sensor_msgs::JointState joint);
        void distCallback(senseglove_shared_resources::FingerDistanceFloats msg);
        void forceCallback_ff(gazebo_msgs::ContactsState msg);
        void forceCallback_mf(gazebo_msgs::ContactsState msg);
        void forceCallback_lf(gazebo_msgs::ContactsState msg);
        void forceCallback_rf(gazebo_msgs::ContactsState msg);
        void forceCallback_th(gazebo_msgs::ContactsState msg);
        
        AllowedRobot selected_robot;
        std::unique_ptr<senseglove::SenseGloveSetup> setup; //setup for senseglove robot

        bool initial; //initial check

        double finger_st0[4]; //  pinky_st0, ring_st0, middle_st0, index_st0 //0 for dip
        double finger_st3[4];
        double finger_bend0[4];
        double finger_bend3[4];

        double hand_relax[15];
        double hand_fist[15];
};

#endif