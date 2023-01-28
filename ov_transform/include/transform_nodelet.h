#ifndef ovtransform_nodelet_CLASS_SRC_ovtransform_nodelet_CLASS_H_
#define ovtransform_nodelet_CLASS_SRC_ovtransform_nodelet_CLASS_H_
#include <nodelet/nodelet.h>
#include "../include/transform.h"

using namespace ov_msckf;
using namespace ov_core;
using namespace ov_type;
using namespace ov_init;

namespace ovtransform_nodelet_ns
{
class OvtransformNodeletClass : public nodelet::Nodelet
{
public:
    OvtransformNodeletClass();
    ~OvtransformNodeletClass();
    void onInit();
    // ros::Subscriber sub_odomimu;
    void odomCallback(const nav_msgs::Odometry& msg_in);
    void setupTransformationMatrix(std::shared_ptr<ov_core::YamlParser> parser);
    Eigen::Matrix<double, 7, 1> print_tf(Eigen::Matrix4d T);
    ros::Subscriber sub_odomimu;
    ros::Publisher pub_odomworldB0;
    ros::Publisher pub_odomworld;
private:
    std::string config_path;
    //std::shared_ptr<ov_core::YamlParser>  parser = std::make_shared<ov_core::YamlParser>(config_path);
    // ov_core::ImuData imu_message;

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
} // namespace transform_nodelet_ns

#endif /* ovmsckf_nodelet_CLASS_SRC_ovmsckf_nodelet_CLASS_H_ */
