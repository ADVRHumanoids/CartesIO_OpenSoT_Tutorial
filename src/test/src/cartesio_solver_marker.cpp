#include <cartesian_interface/CartesianInterfaceImpl.h>
#include <XBotInterface/RobotInterface.h>
#include <thread>

#include <ros/ros.h>

#include <interactive_markers/interactive_marker_server.h>

using namespace XBot::Cartesian;

Eigen::Affine3d Tref;
Eigen::Quaterniond quat;

void processFeedback(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
    Tref.translation().x() = feedback->pose.position.x;
    Tref.translation().y() = feedback->pose.position.y;
    Tref.translation().z() = feedback->pose.position.z;
    
    //quat.w()=feedback->pose.orientation.w;
   // quat.x()=feedback->pose.orientation.x;
   // quat.y()=feedback->pose.orientation.y;
   // quat.z()=feedback->pose.orientation.z;
    
    //Tref.linear()=quat.toRotationMatrix();
    
    
  ROS_INFO_STREAM( feedback->marker_name << " is now at "
      << feedback->pose.position.x << ", " << feedback->pose.position.y
      << ", " << feedback->pose.position.z << ", " 
      << feedback->pose.orientation.x << ", " << feedback->pose.orientation.y
      << ", " << feedback->pose.orientation.z <<", " << feedback->pose.orientation.w );
}

int main(int argc, char **argv)
{
    
        
    ros::init(argc, argv, "simple_marker");

    // create an interactive marker server on the topic namespace simple_marker
    interactive_markers::InteractiveMarkerServer server("simple_marker");

    // create an interactive marker for our server
    visualization_msgs::InteractiveMarker int_marker;
    int_marker.header.frame_id = "world";
    int_marker.header.stamp=ros::Time::now();
    int_marker.name = "lf_foot_marker";
    int_marker.scale = 0.3;
    
    // create a grey box marker
    visualization_msgs::Marker box_marker;
    box_marker.type = visualization_msgs::Marker::CUBE;
    box_marker.scale.x = 0.12;
    box_marker.scale.y = 0.12;
    box_marker.scale.z = 0.12;
    box_marker.color.r = 0.5;
    box_marker.color.g = 0.5;
    box_marker.color.b = 0.5;
    box_marker.color.a = 1.0;

    // create a non-interactive control which contains the box
    visualization_msgs::InteractiveMarkerControl box_control;
    box_control.always_visible = true;
    box_control.markers.push_back( box_marker );

    // add the control to the interactive marker
    int_marker.controls.push_back( box_control );

    // an option structure which is needed to make a model
    XBot::ConfigOptions xbot_cfg;

    // set the urdf and srdf path..
    xbot_cfg.set_urdf_path(URDF_PATH);
    xbot_cfg.set_srdf_path(SRDF_PATH);

    // the following call is needed to generate some default joint IDs
    xbot_cfg.generate_jidmap();

    // some additional parameters..
    xbot_cfg.set_parameter("is_model_floating_base", true);
    xbot_cfg.set_parameter<std::string>("model_type", "RBDL");

    // and we can make the model class
    auto model = XBot::ModelInterface::getModel(xbot_cfg);
    std::string path_to_config_file = XBot::Utils::getXBotConfig();
    XBot::RobotInterface::Ptr robot = XBot::RobotInterface::getRobot ( path_to_config_file ); // returns a shared pointer to a robot object

    // initialize to a homing configuration
    Eigen::VectorXd qhome,qhomer;
    model->getRobotState("home", qhome);
    model->setJointPosition(qhome);
    model->update();
    

    // before constructing the problem description, let us build a
    // context object which stores some information, such as
    // the control period
    const double dt = 0.01;
    auto ctx = std::make_shared<Context>(
                std::make_shared<Parameters>(dt),
                model
            );

    // load the ik problem given a yaml file
    auto ik_pb_yaml = YAML::LoadFile(IK_PB_PATH);
    ProblemDescription ik_pb(ik_pb_yaml, ctx);

    // we are finally ready to make the CartesIO solver "OpenSot"
    auto solver = CartesianInterfaceImpl::MakeInstance("OpenSot",
                                                       ik_pb, ctx
                                                       );



    // inspect properties of "left_hand" task
    std::string task_name = "left_hand";
    auto larm = solver->getTask(task_name);

    // check that "left_hand" is actually a Cartesian type task
    auto larm_cartesian = std::dynamic_pointer_cast<CartesianTask>(larm);
    if(!larm_cartesian)
    {
        throw std::runtime_error("Unexpected task type!");
    }

    
    
    larm_cartesian->getPoseReference(Tref);
    quat=Tref.linear();
    
    int_marker.pose.position.x=Tref.translation().x();
    int_marker.pose.position.y=Tref.translation().y();
    int_marker.pose.position.z=Tref.translation().z();
    
    visualization_msgs::InteractiveMarkerControl rotate_control;
    rotate_control.orientation.w = 1*quat.w();
    rotate_control.orientation.x = 1*quat.x();
    rotate_control.orientation.y = 0;
    rotate_control.orientation.z = 0;
    rotate_control.name = "rotate_x";
    rotate_control.interaction_mode =visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(rotate_control);
    
    rotate_control.name = "move_x";
    rotate_control.interaction_mode =visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(rotate_control);
    
    
    rotate_control.orientation.w = 1*quat.w();
    rotate_control.orientation.x = 0;
    rotate_control.orientation.y = 1*quat.y();
    rotate_control.orientation.z = 0;
    rotate_control.name = "rotate_z";
    rotate_control.interaction_mode =visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(rotate_control);
    
    rotate_control.name = "move_z";
    rotate_control.interaction_mode =visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(rotate_control);
    
    rotate_control.orientation.w = 1*quat.w();
    rotate_control.orientation.x = 0;
    rotate_control.orientation.y = 0;
    rotate_control.orientation.z = 1*quat.z();
    rotate_control.name = "rotate_y";
    rotate_control.interaction_mode =visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(rotate_control);
    
    rotate_control.name = "move_y";
    rotate_control.interaction_mode =visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(rotate_control);
  

    double time = 0;
    Eigen::VectorXd q, qdot, qddot;
    Eigen::VectorXd qr, qdotr;

    server.insert(int_marker, &processFeedback);
    server.applyChanges();  

    int trajTime=3;
    
    while (ros::ok())
    {
    if(larm_cartesian->getTaskState() != State::Reaching)
        larm_cartesian->setPoseTarget(Tref, trajTime);
    
    solver->update(time, dt);

    model->getJointPosition(q);
    model->getJointVelocity(qdot);
    model->getJointAcceleration(qddot);

    q += dt * qdot + 0.5 * std::pow(dt, 2) * qddot;
    qdot += dt * qddot;

    model->setJointPosition(q);
    model->setJointVelocity(qdot);
    model->update();
    
    robot->setReferenceFrom(*model);
    robot->getPositionReference(qr);
    robot->getVelocityReference(qdotr);
    
    robot->setPositionReference(qr);
    robot->setVelocityReference(qdotr);
    robot->move();

    // roughly loop at 100 Hz
    std::this_thread::sleep_for(std::chrono::duration<double>(dt));
    time += dt;

    
    // 'commit' changes and send to all clients  
    
    ros::spinOnce();  
    }
        


}
