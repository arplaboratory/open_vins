#ifndef ovtransform_CLASS_SRC_ovtransform_CLASS_H_
#define ovtransform_CLASS_SRC_ovtransform_CLASS_H_

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <iostream>

#include "../../ov_msckf/src/ros/ROS1Visualizer.h"
#include <ros/ros.h>

#include "../../ov_msckf/src/core/VioManager.h"
#include "../../ov_msckf/src/core/VioManagerOptions.h"

using namespace ov_msckf;
using namespace ov_core;
using namespace ov_type;
using namespace ov_init;
namespace ov_transform{

// void setup_Transform_T_MtoW(std::shared_ptr<ov_core::YamlParser> parser) {
//   // Eigen::Matrix4d T_ItoW = Eigen::Matrix4d::Identity();
//   Eigen::Matrix4d T_ItoC = Eigen::Matrix4d::Identity();
//   Eigen::Matrix4d T_CtoB = Eigen::Matrix4d::Identity();
//   parser->parse_external("relative_config_imucam", "cam0", "T_cam_imu", T_ItoC); // T_cam_imu is transformation from IMU to camera coordinates from kalibr
//   parser->parse_external("relative_config_imu", "imu0", "T_cam_body", T_CtoB); // from camera-vicon calibration
// ROS_INFO("<<<Line 26 in transform.h");
// }

class Transform_calculator {
 	public:
 		Transform_calculator(ros::NodeHandle nodeHandle, std::shared_ptr<ov_core::YamlParser> parser);

		void setup();
        ros::Subscriber sub_odomimu;
        ros::Publisher pub_odomworldB0;
        ros::Publisher pub_odomworld;
    
        void odomCallback(const nav_msgs::OdometryPtr& msg_in);
	private:
		
        void setupTransformationMatrix();
        Eigen::Matrix<double, 7, 1> print_tf(Eigen::Matrix4d T);

		
        ros::NodeHandle nh;
        std::shared_ptr<ov_core::YamlParser> parser;
        // Transformation between vio local (IMU) frame to vicon world frame
        Eigen::Matrix4d T_MtoW = Eigen::Matrix4d::Identity();
        Eigen::Matrix4d T_ItoB = Eigen::Matrix4d::Identity();
        Eigen::Matrix4d T_BtoI = Eigen::Matrix4d::Identity();
        Eigen::Matrix4d T_B0toW = Eigen::Matrix4d::Identity();
        Eigen::Matrix4d T_WtoB0 = Eigen::Matrix4d::Identity();
        Eigen::Matrix4d T_MtoB0 = Eigen::Matrix4d::Identity();
        Eigen::Matrix<double, 7, 1> T_MtoW_eigen;

        bool got_init_tf=false;
        Eigen::Matrix4d T_init_tf = Eigen::Matrix4d::Identity();
        Eigen::Matrix4d T_init_tf_inv = Eigen::Matrix4d::Identity();
        Eigen::Matrix3d hat_M2B0;

        int skip_count = 0;

        double pub_frequency = 0.0;
        float imu_rate = 0;
        float odom_rate = 0;
        double last_timestamp = 0;
		
};


}// namespace ovtransform




#endif