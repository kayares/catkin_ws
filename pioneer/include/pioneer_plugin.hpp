#ifndef PIONEER_PLUGIN_H
#define PIONEER_PLUGIN_H

#include <eigen3/Eigen/Dense>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>

#include <string.h>
#include <iostream>
#include <boost/bind.hpp>
#include <vector>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int8.h>
#include <tf2_msgs/TFMessage.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32.h>
#include <ignition/math/Pose3.hh>

using gazebo::physics::ModelPtr;
using gazebo::physics::LinkPtr;
using gazebo::sensors::ImuSensorPtr;
using gazebo::physics::JointPtr;
using gazebo::event::ConnectionPtr;
using gazebo::common::Time;

using Eigen::Vector3d;
using Eigen::Quaterniond;
using Eigen::VectorXd;
using Eigen::MatrixXd;
using Eigen::Matrix3d;
using Eigen::Matrix4d;

using Eigen::Vector3f;
using Eigen::Quaternionf;
using Eigen::VectorXf;
using Eigen::MatrixXf;
using Eigen::Matrix3f;
using Eigen::Matrix4f;


#define PI 3.14159265358979
#define M2R 2*PI/4096
#define DEG2RAD		0.017453292519943
#define RAD2DEG		57.295779513082323
#define G -9.81;
namespace gazebo
{
 class pioneer : public ModelPlugin
 {
  private:
        physics::JointController *jc0;
        ModelPtr model;
        LinkPtr base_link, RL_link1, RL_link2, RL_link3, RL_link4, RL_link5, RL_link6;
        LinkPtr LL_link1, LL_link2, LL_link3, LL_link4, LL_link5, LL_link6; 
        JointPtr RL_j1, RL_j2, RL_j3, RL_j4, RL_j5, RL_j6,LL_j1, LL_j2, LL_j3, LL_j4, LL_j5, LL_j6;
        JointPtr RA_j1,RA_j2,RA_j3,RA_j4,LA_j1,LA_j2,LA_j3,LA_j4,bodyj,Neck_j1,Neck_j2;
        ConnectionPtr updateConnection;

        // ************* sensor variables ****************//
        sensors::ContactSensorPtr RL_Sensor;
        sensors::ContactSensorPtr LL_Sensor;
        sensors::SensorPtr Sensor;
        sensors::ImuSensorPtr ImuSensor;
        msgs::Contacts RL_contacts;
        msgs::Contacts LL_contacts;
        ros::NodeHandle n;
        ros::Subscriber sub;
        ros::Subscriber sub_motion_selector;
        ros::Publisher pub_joint_state;
        ignition::math::Pose3d body_imu_com_info;

        int time;
        int indext = 0; 
        float angle;
        float mode = 0;
        double walkfreq = 1.48114;
	    double walktime = 1 / walkfreq;
        double freq = 500;
        double del_t = 1/freq;
        double simt = walktime * 500;
        double sim_time = 5 * walktime;
	    int sim_n = sim_time * freq;

        MatrixXd ref_RL_th;
        MatrixXd ref_LL_th;
        MatrixXd ref_RL_th0;
        MatrixXd ref_LL_th0;
        MatrixXd ref_RL_th1;
        MatrixXd ref_LL_th1;
        MatrixXd ref_RL_th2;
        MatrixXd ref_LL_th2;
        MatrixXd ref_RL_th3;
        MatrixXd ref_LL_th3;
        MatrixXd ref_RL_th4;
        MatrixXd ref_LL_th4;
        MatrixXd ref_RL_th5;
        MatrixXd ref_LL_th5;
        MatrixXd ref_RL_th6;
        MatrixXd ref_LL_th6;
        MatrixXd ref_RL_th7;
        MatrixXd ref_LL_th7;
        
        VectorXd TurningTrajectory_coeff = VectorXd::Zero(6);
        VectorXd BackTrajectory_coeff = VectorXd::Zero(6);
        VectorXd RL_th = VectorXd::Zero(6);
        VectorXd LL_th = VectorXd::Zero(6);
        VectorXd ref_RA_th = VectorXd::Zero(4);
        VectorXd ref_LA_th = VectorXd::Zero(4);
        VectorXd sensor_th = VectorXd::Zero(23);
        VectorXd error = VectorXd::Zero(23);
        VectorXd error_dot = VectorXd::Zero(23);
        VectorXd prev_position = VectorXd::Zero(23);
        VectorXd torque = VectorXd::Zero(23);

        //about IMU
        ignition::math::Vector3d angularVelocity;
        ignition::math::Vector3d linearAcceleration;
        ignition::math::Quaterniond body_quat;
        MatrixXd body_rotation_matrix = MatrixXd::Zero(3,3);
        double body_roll, body_pitch;


        const std::vector<std::string> joint_names = {"bodyj", "RA_j1", "RA_j2", "RA_j3", "RA_j4", "LA_j1", "LA_j2", "LA_j3", "LA_j4", "Neckj_1", "Neck_j2", "RL_j1", "RL_j2", "RL_j3", "RL_j4", "RL_j5", "RL_j6", "LL_j1", "LL_j2", "LL_j3", "LL_j4", "LL_j5", "LL_j6"};
        common::Time last_update_time;
        common::Time current_time;
        event::ConnectionPtr update_connection;
        double dt;
        
        // double step_time{0};
        // double cnt_time{0};
        // unsigned int cnt{0};
        // unsigned int start_flag{0};
        FILE *all_theta_data;
        FILE *Imu_pos;
        FILE *ref_theta_data;
        int theta_count = 0;
    public:

        pioneer() {}
        ~pioneer()
        {
            this->n.shutdown(); 
        }

        void Load(ModelPtr _model, sdf::ElementPtr);
        void GetLinks();
        void GetJoints();
        void GetSensor();
        void IdleMotion();
        void GetJointPosition();
        void MotionMaker();
        void InitROSPubSetting();

        void OnUpdate(const common::UpdateInfo &);
        void GetSensorValues();
        void ROSMsgPublish();
        void SetJointPosition();
        void PostureGeneration();
        void TurnCallback(const std_msgs::Float32Ptr &msg);
        void SelectMotion(const std_msgs::Float32Ptr &msg);
        void TurningTrajectory_LL(double angle);
        void TurningTrajectory_RL(double angle);
        double Turn(double t);
        double Back(double t);
        void PIDcontroller();
        void SetTorque();
        void MakeMatlabFile();


};
GZ_REGISTER_MODEL_PLUGIN(pioneer);
    
}

#endif