#include "ik.h"
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>

#include <config_server/parameter.h>

class Controller
{
public:
  Controller(string name);
  IK m_ik;
  void publish_msg(vector<double> positions);
  void subscribe_to_sense_glove();

  //TODO: m_ for mebmbers of class & make the SHUNKJOINTS a vector of strings
private:
  string m_name;
  sensor_msgs::JointState m_msg;
  ros::NodeHandle n;
  ros::Timer m_timer;
  ros::Publisher m_hand_joints_controller_node;
  ros::Publisher m_marker_pub;
  ros::Subscriber sense_glove_subscriber;
  tf::TransformListener m_listener;
  visualization_msgs::Marker m_marker;
  vector<string> add_names(vector<string> names);
  void tf_listener(const ros::TimerEvent &event);

  void setupMarker();
  void addMarker(std::string name);
  interactive_markers::InteractiveMarkerServer m_markerServer;
  std::map<std::string, Eigen::Vector3d> m_markerPoses;

  boost::shared_ptr<robot_model_loader::RobotModelLoader> m_moveItLoader;
  robot_model::RobotModelPtr m_robotModel;
  boost::shared_ptr<robot_state::RobotState> m_robotState;
  void handleInteractiveMarkerUpdate(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &fb);

  ros::Subscriber m_sub_js;
  void jsCallback(const sensor_msgs::JointState &msg);
  sensor_msgs::JointState m_msg_js;
  bool m_jsReceived = false;

  config_server::Parameter<bool> m_param_activate_hand;

};

Controller::Controller(string name)
    : m_name(name), m_markerServer("ik_marker")
    , m_param_activate_hand("activate_hand", false)
{
  m_moveItLoader = boost::make_shared<robot_model_loader::RobotModelLoader>("/robot_description");
  m_robotModel = m_moveItLoader->getModel();
  m_robotState.reset(new robot_state::RobotState(m_robotModel));
  m_robotState->setToDefaultValues();

  m_hand_joints_controller_node = n.advertise<sensor_msgs::JointState>("/svh_controller/channel_targets", 1000);
  m_marker_pub = n.advertise<visualization_msgs::MarkerArray>("visualization_marker", 1);
  m_msg.name = add_names(SCHUNKJOINTS);

  m_msg.name.push_back("svh_hand_Ring_Finger");
  m_msg.name.push_back("svh_hand_Pinky");
  m_msg.name.push_back("svh_hand_Finger_Spread");

  m_sub_js = n.subscribe("/joint_states", 1, &Controller::jsCallback, this);

  while (!m_jsReceived)
  {
    ROS_WARN_THROTTLE(1.0, "wait for joint states");
    ros::spinOnce();
  }

  setupMarker();

  m_timer = n.createTimer(ros::Duration(0.1), &Controller::tf_listener, this);
};

void Controller::jsCallback(const sensor_msgs::JointState &msg)
{
  m_msg_js = msg;
  m_jsReceived = true;
}

void Controller::publish_msg(vector<double> positions)
{
  m_msg.header.stamp = ros::Time::now();
  m_msg.position = positions;
  //cout << positions.size() << endl;
  // for (int i =0; i < positions.size();++i)
  // {
  //   cout << "position = " << m_msg.position[i] << endl;
  //   cout << "name = " << m_msg.name[i] << endl;
  // }

  m_msg.position.push_back(0);
  m_msg.position.push_back(0);
  m_msg.position.push_back(0);

  m_hand_joints_controller_node.publish(m_msg);
}

vector<string> Controller::add_names(vector<string> names)
{

  for (int i = 0; i < names.size(); ++i)
  {
    names[i] = m_name + "_hand_" + names[i];
  }



  return names;
}

void Controller::subscribe_to_sense_glove()
{
  ros::spin();
}

void Controller::tf_listener(const ros::TimerEvent &event)
{
  bool use_glove = true;
  vector<Eigen::Vector3d> tipPositions(3);

  if (use_glove)
  {

    //cout << "calling tf_listener" << endl;

    array<tf::StampedTransform, 3> transform_array;
    try
    {
      m_listener.lookupTransform( "glove_base_link","glove_thumb_finger_fingertip_link",
                                 ros::Time(0), transform_array[0]);
      m_listener.lookupTransform("glove_base_link", "glove_index_finger_fingertip_link",
                                 ros::Time(0), transform_array[1]);
      m_listener.lookupTransform("glove_base_link", "glove_middle_finger_fingertip_link",
                                 ros::Time(0), transform_array[2]);
      // m_listener.lookupTransform("glove_base_link", "glove_ring_finger_fingertip_link",
      //                            ros::Time(0), transform_array[3]);
      // m_listener.lookupTransform("glove_base_link", "glove_pinky_finger_fingertip_link",
      //                            ros::Time(0), transform_array[4]);
    }
    catch (tf::TransformException &ex)
    {
      ROS_ERROR("%s", ex.what());
      ros::Duration(1.0).sleep();
    }

    //Eigen::Vector3d thumb_tip;
    Eigen::Vector3d correctionOffset(0,-0.01,0.05);

    for(int i=0;i<3;i++)
    {
        tf::vectorTFToEigen(transform_array[i].getOrigin(), tipPositions[i]);

        tipPositions[i] += correctionOffset;
    }

    tipPositions[0] += Eigen::Vector3d(0.03,0.04,0.04);



  }
  else
  {
    tipPositions[0] = m_markerPoses["thumb"];
    tipPositions[1] = m_markerPoses["index"];
    tipPositions[2] = m_markerPoses["middle"];
    // tipPositions[3] = m_markerPoses["ring"];
    // tipPositions[4] = m_markerPoses["pinky"];
  }

  //visualize goal positions
  visualization_msgs::MarkerArray msgArray;
  for(unsigned int i=0;i<tipPositions.size();i++)
  {
    visualization_msgs::Marker marker;
    marker.id = i;
    marker.header.frame_id = "base_link";
    marker.header.stamp = ros::Time::now();

    marker.type = visualization_msgs::Marker::SPHERE;
    marker.scale.x = marker.scale.y = marker.scale.z = 0.01;
    marker.color.a = 1.0;
    marker.color.g = 1.0 - i / (float)tipPositions.size();
    marker.color.r = i / (float)tipPositions.size();
    marker.pose.orientation.w = 1.0;

    tf::pointEigenToMsg(tipPositions[i], marker.pose.position);

    msgArray.markers.push_back(marker);

  }
    m_marker_pub.publish(msgArray);

  Eigen::VectorXd returnedJoints = m_ik.solveIK(tipPositions);
  vector<double> positions;
  for (int i = 0; i < returnedJoints.size(); ++i)
  {
    positions.push_back(returnedJoints[i]);
    //cout << positions[i] << endl;
  }

  if(m_param_activate_hand())
      publish_msg(positions);
}

void Controller::setupMarker()
{
  addMarker("thumb");
  addMarker("index");
  addMarker("middle");
  // addMarker("ring");
  // addMarker("pinky");
}

void Controller::addMarker(std::string name)
{
  m_robotState->setVariableValues(m_msg_js);
  m_robotState->updateLinkTransforms();

  auto tipLink = m_robotModel->getLinkModel("svh_hand_" + name + "_fingertip_link");
  Eigen::Vector3d tipPosition = m_robotState->getGlobalLinkTransform(tipLink).translation();

  visualization_msgs::InteractiveMarker interaktiveMarker;
  interaktiveMarker.name = name;
  interaktiveMarker.header.frame_id = "base_link";
  interaktiveMarker.scale = 0.03f;
  interaktiveMarker.controls.resize(2);
  interaktiveMarker.controls[0].name = "move";
  interaktiveMarker.controls[0].interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_3D;
  interaktiveMarker.controls[0].always_visible = true;
  interaktiveMarker.controls[1].name = "icon";
  interaktiveMarker.controls[1].interaction_mode = visualization_msgs::InteractiveMarkerControl::NONE;
  interaktiveMarker.controls[1].markers.resize(1);
  interaktiveMarker.controls[1].markers[0].id = 0;
  interaktiveMarker.controls[1].markers[0].type = visualization_msgs::Marker::SPHERE;
  interaktiveMarker.controls[1].markers[0].color.r = 1.0;
  interaktiveMarker.controls[1].markers[0].color.g = 1.0;
  interaktiveMarker.controls[1].markers[0].color.b = 0.0;
  interaktiveMarker.controls[1].markers[0].color.a = 1.0;
  interaktiveMarker.controls[1].markers[0].scale.x = 0.01;
  interaktiveMarker.controls[1].markers[0].scale.y = 0.01;
  interaktiveMarker.controls[1].markers[0].scale.z = 0.01;
  m_markerPoses[name] = tipPosition;

  tf::pointEigenToMsg(tipPosition, interaktiveMarker.pose.position);

  {
    visualization_msgs::InteractiveMarkerControl control;
    control.orientation.w = 1;
    control.orientation.x = 1;
    control.orientation.y = 0;
    control.orientation.z = 0;
    control.name = "move_x";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    interaktiveMarker.controls.push_back(control);

    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 1;
    control.orientation.z = 0;
    control.name = "move_z";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    interaktiveMarker.controls.push_back(control);

    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 0;
    control.orientation.z = 1;
    control.name = "move_y";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    interaktiveMarker.controls.push_back(control);
  }
  m_markerServer.insert(interaktiveMarker, boost::bind(&Controller::handleInteractiveMarkerUpdate, this, _1));
  m_markerServer.applyChanges();
}

void Controller::handleInteractiveMarkerUpdate(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &fb)
{
  //     if(fb->event_type == visualization_msgs::InteractiveMarkerFeedback::MOUSE_DOWN)

  Eigen::Vector3d position;
  tf::pointMsgToEigen(fb->pose.position, position);

  m_markerPoses[fb->marker_name] = position;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "hand_controller");

  Controller hand_controller("svh");

  ros::spin();

  return 0;
}
