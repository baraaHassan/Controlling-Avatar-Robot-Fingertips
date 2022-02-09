// Hand IK

#include "hand_ik.h"

#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>


// static const std::array<std::string, 5> FINGER = {"thumb", "index", "middle" , "ring", "pinky"};
static const std::array<std::string, 3> FINGER = {"thumb", "index", "middle"};

HandIK::HandIK()
: m_nh("~")
, m_markerServer("ik_marker")
, m_param_use_glove("use_glove", false)
, m_param_activate_hand("activate_hand", false)
{
    m_moveItLoader = boost::make_shared<robot_model_loader::RobotModelLoader>("/robot_description");
    m_robotModel = m_moveItLoader->getModel();
    m_robotState.reset(new robot_state::RobotState(m_robotModel));
    m_robotState->setToDefaultValues();



    m_timer = m_nh.createTimer(ros::Duration(0.2), &HandIK::timerCB, this, false, false);
    m_pub_cmd = m_nh.advertise<sensor_msgs::JointState>("/svh_controller/channel_targets", 1);
    m_sub_js = m_nh.subscribe("/joint_states", 1, &HandIK::jsCallback, this);
    m_pub_marker = m_nh.advertise<visualization_msgs::MarkerArray>("goalMarker", 1);

	while(!m_jsReceived)
	{
		ROS_WARN_THROTTLE(1.0, "wait for joint states");
		ros::spinOnce();
	}

	setupMarker();

    for(auto& fingerName : FINGER)
    {
        Finger f;
        setFingerData(f, fingerName);
        m_fingers.push_back(f);
    }

	m_timer.start();
}



void HandIK::timerCB(const ros::TimerEvent&)
{
    //set goal positions
    if(m_param_use_glove())
    {
        m_robotState->setVariableValues(m_msg_js);
        m_robotState->updateLinkTransforms();

        Eigen::Vector3d gloveBasePosition = m_robotState->getGlobalLinkTransform("glove_base_link").translation();

        Eigen::Vector3d goalOffset(0.06,0.02,0.02);

        for(auto& f : m_fingers)
        {
            std::string tipName = "glove_" + f.name + "_finger_fingertip_link";
            Eigen::Vector3d tipPosition = m_robotState->getGlobalLinkTransform(tipName).translation();
            f.goalPose = tipPosition - gloveBasePosition;
            f.goalPose += goalOffset;
        }
        //thumb
        m_fingers[0].goalPose += Eigen::Vector3d(0.06,0.02,0.02);
    }
    else
    {
        for(auto& f : m_fingers)
        {
            f.goalPose = m_markerPoses.at(f.name);
        }

    }


    //Viualize goal positions
    visualization_msgs::MarkerArray msgArray;
    for(unsigned int i=0;i<m_fingers.size();i++)
    {
        visualization_msgs::Marker marker;
        marker.id = i;
        marker.header.frame_id = "base_link";
        marker.header.stamp = ros::Time::now();

        marker.type = visualization_msgs::Marker::SPHERE;
        marker.scale.x = marker.scale.y = marker.scale.z = 0.01;
        marker.color.a = 1.0;
        marker.color.g = 1.0 - i / (float)m_fingers.size();
        marker.color.r = i / (float)m_fingers.size();
        marker.pose.orientation.w = 1.0;

        tf::pointEigenToMsg(m_fingers[i].goalPose, marker.pose.position);

        msgArray.markers.push_back(marker);

    }
    m_pub_marker.publish(msgArray);


    std::map<std::string, double> cmd = solveIK();

    if(m_param_activate_hand())
    {
        sensor_msgs::JointState msg_cmd;
        msg_cmd.header.stamp = ros::Time::now();
        for(auto& it : cmd)
        {
            msg_cmd.name.push_back(it.first);
            msg_cmd.position.push_back(it.second);
        }

        m_pub_cmd.publish(msg_cmd);
    }
}

std::map<std::string, double> HandIK::solveIK()
{

    const double EPS_LINEAR = 1e-4;
    const int MAX_ITERATIONS = 100;
    const double CLAMP_TRANSLATION = 0.05;
    const double alp = 0.02;


    //initialize with the current joint state
    m_robotState->updateLinkTransforms();
    std::vector<double> js;
    std::vector<std::string> qNames;
    std::map<std::string, int> qJointId;


    std::vector<const robot_model::JointModel*> activeJoints;

    int i=0;
    for(auto& activeJoint : m_robotModel->getActiveJointModels())
    {
        std::string jName = activeJoint->getName();
        if(jName.substr(0,8) != "svh_hand")
            continue;

        js.push_back(*m_robotState->getJointPositions(jName));
        qNames.push_back(jName);
        qJointId[jName] = i;
        i++;
        activeJoints.push_back(activeJoint);
    }


    Eigen::VectorXd q = Eigen::VectorXd::Map(js.data(), js.size());
    Eigen::VectorXd qDiff(q.size());

    int iteration = 0;

    //you can use this to measure the runtime here
    // 	ros::WallTime start = ros::WallTime::now();
    //time elapsed in seconds. Move this at the end of the algorithm
    //  double sec = (ros::Time::now() - start).toSec();

    while(iteration < MAX_ITERATIONS)
    {
        iteration++;


        //Set robot model to current state
        std::map<std::string, double> qMap;
        for(unsigned i=0;i<q.size();i++)
        {
            qMap[qNames[i]] = q[i];
        }
        m_robotState->setVariablePositions(qMap);
        m_robotState->updateLinkTransforms();

        //check if we are done
        bool done = true;
        for(auto& finger : m_fingers)
        {

            // Calculate current endeffector pose
            finger.currentPose = m_robotState->getGlobalLinkTransform(finger.tipLink).translation();

            // Calculate pose difference
            finger.poseDiff = finger.goalPose - finger.currentPose;
            if(finger.poseDiff.squaredNorm() > CLAMP_TRANSLATION*CLAMP_TRANSLATION)
                finger.poseDiff = CLAMP_TRANSLATION * finger.poseDiff / finger.poseDiff.norm();

            if(finger.poseDiff.squaredNorm() > EPS_LINEAR*EPS_LINEAR)
                done = false;
        }

        if(done)
        {
            return qMap;
        }


        //reset qDiff
        qDiff.setZero();

        for(auto& finger : m_fingers)
        {
            // Calculate the Jacobian matrix
            Eigen::MatrixXd J = jacobian(finger);

            //Pseudo inverse
            Eigen::MatrixXd Jinv = pinv(J);

            //get thetaDiff
            finger.thetaDiff = Jinv * finger.poseDiff;

            //update qDiff
            for(unsigned j=0;j<finger.joints.size();j++)
            {
                //check if mimic joint

                auto mimic = finger.joints[j]->getMimic();
                if(!mimic)
                {
                    //active joint
                    std::string jointName = finger.joints[j]->getName();
                    qDiff[qJointId[jointName]] += finger.thetaDiff[j];
                }
                else
                {
                    std::string parentJointName = mimic->getName();
                    double theta = finger.thetaDiff[j] / finger.joints[j]->getMimicFactor();
                    qDiff[qJointId[parentJointName]] += theta;
                }
            }
        }


        //apply scaled changes to current joint angles. Scale with alp(see above)
        q += alp * qDiff;

		bool ok = true;
		for(auto& activeJoint : activeJoints)
        {
            std::string jName = activeJoint->getName();
            int jIdx = qJointId.at(jName);
			double previous = q[jIdx];
			activeJoint->enforcePositionBounds(&q[jIdx]);

			if(!std::isfinite(previous) || !std::isfinite(q[jIdx]) || std::abs(previous - q[jIdx]) > 0.5)
			{
				ROS_WARN("Strange behavior on enforcePositionBounds for joint %s", jName.c_str());
				ROS_WARN("previous: %f", previous);
				ROS_WARN("currentJointAngles[i]: %f", q[jIdx]);
// 				ROS_WARN("start[i]: %f", startJointAngles[i]);

                // This allows to find the closest configuration when the problem is in kinematics limits
                if( std::abs(previous - q[jIdx]) > 0.5)
                    ok = true;
                else
                    ok = false;
			}
		}

		if(!ok)
		{
			ROS_ERROR("Happened in iteration %d", iteration);
// 			ROS_ERROR_STREAM("J\n" << J);
// 			ROS_ERROR_STREAM("invModJ\n" << invModifiedJ);
// 			ROS_ERROR_STREAM("poseDiff: " << poseDiff.transpose());
// 			ROS_ERROR_STREAM("currentPose:\n" << currentPose.matrix());
// 			ROS_ERROR_STREAM("targetPose:\n" << targetPose.matrix());
// 			ROS_ERROR_STREAM("thetaDiff: " << thetaDiff.transpose());
			break;
        }
    }

//     ROS_WARN("MAX NUMBER OF ITERATIONS '%u' IS REACHED", iteration);
    std::map<std::string, double> ret;
    for(unsigned i=0;i<q.size();i++)
    {
        ret[qNames[i]] = q[i];
    }
    return ret;
}


void HandIK::setFingerData(Finger& finger, std::string name)
{
    finger.name = name;
    std::string tipFrame = "svh_hand_" + name + "_fingertip_link";

    finger.tipLink = m_robotModel->getLinkModel(tipFrame);

    const robot_model::LinkModel* link = finger.tipLink;
    while(link->getName() != "base_link")
    {
        auto parentJoint = link->getParentJointModel();
        if(parentJoint->getType() == robot_model::JointModel::FIXED)
        {
            ROS_DEBUG("Fixed joint found '%s' ", parentJoint->getName().c_str());
        }
        else
        {
            finger.jointNames.push_back(parentJoint->getName());
            finger.joints.push_back(parentJoint);
            auto revJoint = dynamic_cast<const robot_model::RevoluteJointModel*>(parentJoint);
            finger.jointAxes.push_back(revJoint->getAxis());
        }

        link = parentJoint->getParentLinkModel();
        if(!link)
        {
            ROS_ERROR("Frame 'base_link' is no ancestor frame of '%s'", tipFrame.c_str()
            );
            throw std::runtime_error("frame error");
        }
    }
}



Eigen::MatrixXd HandIK::jacobian(Finger& finger) const
{
    int numJoints = finger.joints.size();
    int numDims = 3;

    Eigen::MatrixXd J(numDims, numJoints);
    J.setZero();

    Eigen::Isometry3d tipPose = m_robotState->getGlobalLinkTransform(finger.tipLink);

    for(int i=0 ; i < numJoints; i++)
    {
        auto joint = finger.joints[i];
        auto parentLink = joint->getParentLinkModel();

        auto jointPose = m_robotState->getGlobalLinkTransform(parentLink) * joint->getChildLinkModel()->getJointOriginTransform();

        Eigen::Affine3d tipToJoint = jointPose.inverse() * tipPose;

        J.col(i) = jointPose.linear() * finger.jointAxes[i].cross(tipToJoint.translation());
    }
    return J;
}



void HandIK::setupMarker()
{
    for(auto f : FINGER)
        addMarker(f);
}

void HandIK::addMarker(std::string name)
{
	m_robotState->setVariableValues(m_msg_js);
	m_robotState->updateLinkTransforms();

	auto tipLink = m_robotModel->getLinkModel("svh_hand_" + name + "_fingertip_link");
	Eigen::Vector3d tipPosition = m_robotState->getGlobalLinkTransform(tipLink).translation();

    m_markerPoses[name] = tipPosition;

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
    m_markerServer.insert(interaktiveMarker, boost::bind(&HandIK::handleInteractiveMarkerUpdate, this, _1));
    m_markerServer.applyChanges();
}




void HandIK::handleInteractiveMarkerUpdate(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& fb)
{
	Eigen::Vector3d position;
    tf::pointMsgToEigen(fb->pose.position, position);

	m_markerPoses[fb->marker_name] = position;
}





void HandIK::jsCallback(const sensor_msgs::JointState& msg)
{
    m_msg_js = msg;
	m_jsReceived = true;
}



Eigen::MatrixXd HandIK::pinv(const Eigen::MatrixXd& J)
{
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(J, Eigen::ComputeFullU | Eigen::ComputeFullV);

    double pinvtol = 1.0e-6;

    Eigen::VectorXd sigma = svd.singularValues();
    for(int i = 0; i < sigma.size(); ++i)
    {
        if(sigma[i] > pinvtol)
            sigma[i] = 1.0 / sigma[i];
        else
            sigma[i] = 0.0;
    }

    return svd.matrixV() * sigma.asDiagonal() * svd.matrixU().transpose();
}




int main(int argc, char** argv)
{

    ros::init(argc, argv, "hand_ik");
    ros::NodeHandle nh;

    HandIK handIk;

    ros::spin();

    return 0;
}
