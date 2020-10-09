// Hand IK
// Author: Christian Lenz <lenz@ais.uni-bonn.de>

#ifndef HAND_IK_H
#define HAND_IK_H


#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/kinematics_base/kinematics_base.h>
#include <Eigen/Dense>

#include <interactive_markers/interactive_marker_server.h>
#include <visualization_msgs/InteractiveMarkerFeedback.h>

#include <config_server/parameter.h>

struct Finger
{
    std::string name;
    std::vector<const robot_model::JointModel*> joints;
    std::vector<Eigen::Vector3d> jointAxes;
    std::vector<std::string> jointNames;
    Eigen::VectorXd thetaDiff;

    robot_model::LinkModel* tipLink;
    Eigen::Vector3d goalPose;
    Eigen::Vector3d currentPose;
    Eigen::Vector3d poseDiff;
};


class HandIK
{
public:
    HandIK();


private:

    ros::NodeHandle m_nh;

    //robot model
    boost::shared_ptr<robot_model_loader::RobotModelLoader> m_moveItLoader;
    robot_model::RobotModelPtr m_robotModel;
    boost::shared_ptr<robot_state::RobotState> m_robotState;

    //publisher & subscriber
    ros::Publisher m_pub_cmd;
    ros::Subscriber m_sub_js;
    ros::Publisher m_pub_marker;

    //Timer
    ros::Timer m_timer;
    void timerCB(const ros::TimerEvent&);

    //Ik
    Eigen::MatrixXd jacobian(Finger& finger) const;
    Eigen::MatrixXd pinv (const Eigen::MatrixXd& J);

    std::map<std::string, double> solveIK();


    //Finger data
    std::vector<Finger> m_fingers;
    void setFingerData(Finger& finger, std::string name);

    //joint state
    void jsCallback(const sensor_msgs::JointState& msg);
    sensor_msgs::JointState m_msg_js;
	bool m_jsReceived = false;

    //interactive marker
    void setupMarker();
    void addMarker(std::string name);
    interactive_markers::InteractiveMarkerServer m_markerServer;

    void handleInteractiveMarkerUpdate(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& fb);

    std::map<std::string,Eigen::Vector3d> m_markerPoses;

    config_server::Parameter<bool> m_param_use_glove;
    config_server::Parameter<bool> m_param_activate_hand;


};


#endif
