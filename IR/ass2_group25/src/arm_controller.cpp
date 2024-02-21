// ROS headers
#include <ros/ros.h>

// MoveIt! headers
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>

// Std C++ headers
#include <string>
#include <vector>
#include <map>
#include <unistd.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "ass2_group25/pick.h"
#include "ass2_group25/place.h"
#include <ass2_group25/objects.h>

#include <gazebo_ros_link_attacher/Attach.h>
#include <gazebo_ros_link_attacher/AttachRequest.h>
#include <gazebo_ros_link_attacher/AttachResponse.h>

//////////////////////
// COLLISION TABLES //
//////////////////////
moveit_msgs::CollisionObject table_s; // squared table
// collision objects vector
std::vector<moveit_msgs::CollisionObject> collision_objects;
// safe pose
geometry_msgs::PoseStamped safe_pose;

// spin function
void spin(){
	ros::Rate rate(15); 
	while (ros::ok()) {
		ros::spinOnce();
		rate.sleep();
	}
}

// open gripper
void openGripper() {
    ros::NodeHandle nh;
    ros::Publisher gripper_publisher = nh.advertise<trajectory_msgs::JointTrajectory>("/gripper_controller/command", 3);
    for(int i=0;i<3;i++) {
        trajectory_msgs::JointTrajectory trajectory_points;
        trajectory_points.joint_names.push_back("gripper_left_finger_joint");
        trajectory_points.joint_names.push_back("gripper_right_finger_joint");
        trajectory_points.points.resize(1);
        trajectory_points.points[0].positions.resize(2);
        trajectory_points.points[0].positions[0] = 0.044;
        trajectory_points.points[0].positions[1] = 0.044;
        trajectory_points.points[0].time_from_start = ros::Duration(1);
        gripper_publisher.publish(trajectory_points);
        sleep(1.1);
    }
}

// close gripper
void closeGripper() {
    ros::NodeHandle nh;
    ros::Publisher gripper_publisher = nh.advertise<trajectory_msgs::JointTrajectory>("/gripper_controller/command", 3);
    for(int i=0;i<3;i++) {
        trajectory_msgs::JointTrajectory trajectory_points;
        trajectory_points.joint_names.push_back("gripper_left_finger_joint");
        trajectory_points.joint_names.push_back("gripper_right_finger_joint");
        trajectory_points.points.resize(1);
        trajectory_points.points[0].positions.resize(2);
        trajectory_points.points[0].positions[0] = 0.0;
        trajectory_points.points[0].positions[1] = 0.0;
        trajectory_points.points[0].time_from_start = ros::Duration(1);
        gripper_publisher.publish(trajectory_points);
        sleep(1.1);
    }
}

// collision tables
void addCollisionTables() {
    ///////////////////////
    // ADD SQUARED TABLE //
    ///////////////////////
    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = "map";
    collision_object.id = "table0";
    // table dimensions
    shape_msgs::SolidPrimitive table;
    table.type = table.BOX;
    table.dimensions.resize(3);
    table.dimensions[0] = 0.913;
    table.dimensions[1] = 0.913;
    table.dimensions[2] = 0.82; // 0.04 + 0.74 + 0.04
    // table pose
    geometry_msgs::Pose table_pose;
    table_pose.orientation.w = 1.0;
    table_pose.position.x = 7.824;
    table_pose.position.y = -2.982;
    table_pose.position.z = table.dimensions[2]/2 - 0.03;
    collision_object.primitives.push_back(table);
    collision_object.primitive_poses.push_back(table_pose);
    collision_object.operation = collision_object.ADD;
    table_s = collision_object;
}

// collision objects
void addCollisionObjects(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface, std::vector<ass2_group25::objects> objs) {
    /////////////////
    // ADD OBJECTS //
    /////////////////
    for(int i=0; i<objs.size(); i++) {
        moveit_msgs::CollisionObject collision_object;
        collision_object.header.frame_id = "base_footprint";
        collision_object.id = std::to_string(objs[i].id);
        // object dimensions
        shape_msgs::SolidPrimitive object;
        if(objs[i].id==2 || objs[i].id==3)
            object.type = object.BOX;
        else
            object.type = object.CYLINDER;
        
        // red
        if(objs[i].id==3) {
            object.dimensions.push_back(0.05);
            object.dimensions.push_back(0.05);
            object.dimensions.push_back(0.05);
        } else if(objs[i].id==2) { // green
            object.dimensions.push_back(0.05);
            object.dimensions.push_back(0.05);
            object.dimensions.push_back(0.05);
        } else if(objs[i].id==1) { // blue
            object.dimensions.push_back(0.1);
            object.dimensions.push_back(0.03);
        } else { // obstacles
            object.dimensions.push_back(0.25);
            object.dimensions.push_back(0.05);
        }
        // object pose
        geometry_msgs::Pose object_pose;
        
        object_pose = objs[i].position.pose;
        // change pose of green object to make it easier to pick
        if(objs[i].id==2) {
            geometry_msgs::Quaternion prev = objs[i].position.pose.orientation;
            tf2::Quaternion q;
            q.setRPY(0, 0, 0);
            objs[i].position.pose.orientation = tf2::toMsg(q);
            objs[i].position.pose.orientation.z = prev.z;
            objs[i].position.pose.position.z += 0.03;
        }
        
        collision_object.primitives.push_back(object);
        collision_object.primitive_poses.push_back(object_pose);
        
        ROS_INFO_STREAM("Object " << collision_object.id << " position: " << collision_object.primitive_poses[0].position.x << " " << collision_object.primitive_poses[0].position.y << " " << collision_object.primitive_poses[0].position.z);
        collision_object.operation = collision_object.ADD;
        collision_objects.push_back(collision_object);
    }
}

// delete collision objects at the end of the pick and place or when an error occurs
void delete_objects(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface) {
    for(int i=0; i<collision_objects.size(); i++) {
        collision_objects[i].operation = collision_objects[i].REMOVE;
    }
    planning_scene_interface.applyCollisionObjects(collision_objects);
    collision_objects.clear();
}

bool pick(ass2_group25::pick::Request &req, ass2_group25::pick::Response &res) {
    ROS_INFO("Ready to pick");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    // pick pose
    geometry_msgs::PoseStamped goal_pose;
    std::string object_ids_to_rm;
    tf2::Quaternion q;
    goal_pose.header.frame_id = "base_footprint";
    // extracting pick pose
    for(int i=0; i<req.detection.size(); i++) {
        ass2_group25::objects obj = req.detection[i];
        // saving pose of the object to pick
        if(obj.id==1 || obj.id==2 || obj.id==3) {
            object_ids_to_rm = std::to_string(obj.id);
            // change orientation of green object
            if(obj.id==2) {
                geometry_msgs::Quaternion prev = obj.position.pose.orientation;
                q.setRPY(0, 0, 0);
                obj.position.pose.orientation = tf2::toMsg(q);
                obj.position.pose.orientation.z = prev.z;
                obj.position.pose.position.z += 0.02;
            }

            goal_pose.pose = obj.position.pose;
            goal_pose.pose.position.z += 0.22;
            q.setRPY(1.5, 1.5, 0);
            goal_pose.pose.orientation = tf2::toMsg(q);
            break;
        }
        ROS_INFO_STREAM("Object " << obj.id << " position: " << obj.position.pose.position.x << " " << obj.position.pose.position.y << " " << obj.position.pose.position.z);
    }
    geometry_msgs::PoseStamped pre_grasp;
    pre_grasp = goal_pose;
    // set pre-grasp pose
    pre_grasp.pose.position.z += 0.04;
    ROS_INFO_STREAM("Goal-grasp pose: " << goal_pose.pose.position.x << " " << goal_pose.pose.position.y << " " << goal_pose.pose.position.z);
    ROS_INFO_STREAM("Pre-grasp pose: " << pre_grasp.pose.position.x << " " << pre_grasp.pose.position.y << " " << pre_grasp.pose.position.z);
    // adding collision objects
    addCollisionTables();
    collision_objects.push_back(table_s);
    addCollisionObjects(planning_scene_interface, req.detection);
    planning_scene_interface.applyCollisionObjects(collision_objects);
    
    // wait a bit for ROS things to initialize
    ros::WallDuration(1.0).sleep();

    moveit::planning_interface::MoveGroupInterface group_arm_torso("arm_torso");
    // choose your preferred planner
    group_arm_torso.setPlannerId("SBLkConfigDefault");
    ////////////////////////
    // PRE-GRASP MOVEMENT //
    ////////////////////////
    group_arm_torso.setPoseReferenceFrame("base_footprint");
    group_arm_torso.setPoseTarget(pre_grasp);
    ROS_INFO_STREAM("Planning to move " <<
                    group_arm_torso.getEndEffectorLink() << " to a target pose expressed in " <<
                    group_arm_torso.getPlanningFrame());

    group_arm_torso.setStartStateToCurrentState();
    group_arm_torso.setMaxVelocityScalingFactor(1.0);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    // set maximum time to find a plan
    group_arm_torso.setPlanningTime(10.0);
    bool success = bool(group_arm_torso.plan(my_plan));
    
    if ( !success ) {
        delete_objects(planning_scene_interface);
        throw std::runtime_error("No plan found for pregrasp pose");
    }

    ROS_INFO_STREAM("Plan found for pre-gasp in " << my_plan.planning_time_ << " seconds");
    // execute the plan
    ros::Time start = ros::Time::now();

    ros::WallDuration(1.0).sleep();

    openGripper();
    ros::WallDuration(1.0).sleep();
    // execute the plan
    group_arm_torso.move();
    ros::WallDuration(1.0).sleep();
    // remove object to pick from collision objects
    for(int i=0; i<collision_objects.size(); i++) {
        if(collision_objects[i].id.compare(object_ids_to_rm)==0)
            collision_objects[i].operation = collision_objects[i].REMOVE;
    }
    planning_scene_interface.applyCollisionObjects(collision_objects);
    
    ///////////////////
    // GOAL MOVEMENT //
    ///////////////////
    group_arm_torso.setPoseTarget(goal_pose);
    success = bool(group_arm_torso.plan(my_plan));
    if ( !success ) {
        delete_objects(planning_scene_interface);
        throw std::runtime_error("No plan found for goal pose");
    }

    ROS_INFO_STREAM("Plan found for goal in " << my_plan.planning_time_ << " seconds");
    // execute the plan
    ros::WallDuration(1.0).sleep();
    group_arm_torso.move();

    ros::WallDuration(1.0).sleep();
    // attach the objects using gazebo_ros_link_attacher service
    ros::NodeHandle nh;
    ros::ServiceClient attach_client = nh.serviceClient<gazebo_ros_link_attacher::Attach>("/link_attacher_node/attach");
    // wait for service to be available
    attach_client.waitForExistence();
    gazebo_ros_link_attacher::Attach attach_srv;
    // attaching links
    attach_srv.request.model_name_1 = "tiago";
    attach_srv.request.link_name_1 = "arm_7_link";
    if(object_ids_to_rm.compare("1")==0) {
        attach_srv.request.model_name_2 = "Hexagon";
        attach_srv.request.link_name_2 = "Hexagon_link";
    }
    else if(object_ids_to_rm.compare("2")==0) {
        attach_srv.request.model_name_2 = "Triangle";
        attach_srv.request.link_name_2 = "Triangle_link";
    }
    else if(object_ids_to_rm.compare("3")==0) {
        attach_srv.request.model_name_2 = "cube";
        attach_srv.request.link_name_2 = "cube_link";
    }
    if(!attach_client.call(attach_srv))
        ROS_ERROR("Failed to call service attach");
    ros::WallDuration(1.0).sleep();
    // close gripper
    closeGripper();
    ros::WallDuration(1.0).sleep();
    //////////////////////////
    // POSTGRASP-1 MOVEMENT //
    //////////////////////////
    geometry_msgs::PoseStamped post_grasp_1;
    post_grasp_1 = pre_grasp;
    post_grasp_1.pose.position.z += 0.05;
    group_arm_torso.setPoseTarget(post_grasp_1);
    success = bool(group_arm_torso.plan(my_plan));
    if ( !success ) {
        delete_objects(planning_scene_interface);
        throw std::runtime_error("No plan found for postgrasp-1 pose");
    }
    ROS_INFO_STREAM("Plan found for post1 in " << my_plan.planning_time_ << " seconds");
    // execute the plan
    ros::WallDuration(1.0).sleep();
    group_arm_torso.move();
    //////////////////////////
    // POSTGRASP-2 MOVEMENT //
    //////////////////////////
    geometry_msgs::PoseStamped post_grasp_2;
    post_grasp_2 = pre_grasp;
    post_grasp_2.pose.position.z += 0.25;
    q.setRPY(1, -1.5, 0);
    post_grasp_2.pose.orientation = tf2::toMsg(q);
    group_arm_torso.setPoseTarget(post_grasp_2);
    success = bool(group_arm_torso.plan(my_plan));
    if ( !success ) {
        delete_objects(planning_scene_interface);
        throw std::runtime_error("No plan found for postgrasp-2 pose");
    }
    ROS_INFO_STREAM("Plan found for post2 in " << my_plan.planning_time_ << " seconds");
    // execute the plan
    ros::WallDuration(1.0).sleep();
    group_arm_torso.move();
    ////////////////////////
    // SAFE POSE MOVEMENT //
    ////////////////////////
    safe_pose.header.frame_id = "base_footprint";
    safe_pose.pose.position.x = 0.4;
    safe_pose.pose.position.y = 0;
    safe_pose.pose.position.z = 1.45;
    q.setRPY(1, -1.5, 0);
    safe_pose.pose.orientation = tf2::toMsg(q);
    group_arm_torso.setPoseTarget(safe_pose);
    
    success = bool(group_arm_torso.plan(my_plan));
    if ( !success ) {
        delete_objects(planning_scene_interface);
        throw std::runtime_error("No plan found for safepose");
    }
    ROS_INFO_STREAM("Plan found for safe in " << my_plan.planning_time_ << " seconds");
    // execute the plan
    ros::WallDuration(1.0).sleep();
    group_arm_torso.move();
    ros::WallDuration(1.0).sleep();

    spinner.stop();
    res.success = true;
    // removing previous collision objects
    delete_objects(planning_scene_interface);
	return true;
}

bool place(ass2_group25::place::Request &req, ass2_group25::place::Response &res) {
    ROS_INFO("Ready to place");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    // add table to collision objects
    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = "map";
    collision_object.id = "table" + std::to_string(req.id);
    // table dimensions
    shape_msgs::SolidPrimitive table;
    table.type = table.CYLINDER;
    table.dimensions.resize(2);
    table.dimensions[0] = 0.69;
    table.dimensions[1] = 0.21;
    // table pose
    geometry_msgs::Pose table_pose;
    table_pose.orientation.w = 1.0;
    table_pose.position.x = req.position.x;
    table_pose.position.y = req.position.y;
    table_pose.position.z = table.dimensions[0]/2 + 0.02;
    collision_object.primitives.push_back(table);
    collision_object.primitive_poses.push_back(table_pose);
    collision_object.operation = collision_object.ADD;
    collision_objects.push_back(collision_object);

    planning_scene_interface.applyCollisionObjects(collision_objects);
    // pose in which to place the object will be hardcoded since there are no obstacles other than the tables
    geometry_msgs::PoseStamped place_pose;
    place_pose.header.frame_id = "base_footprint";
    place_pose.pose.position.x = 0.8;
    place_pose.pose.position.y = 0;
    place_pose.pose.position.z = collision_objects[0].primitives[0].dimensions[0] + 0.35;
    // orientation in wich the reference frame of the gripper is pointing down
    tf2::Quaternion q;
    q.setRPY(1.5, 1.5, 0);
    place_pose.pose.orientation = tf2::toMsg(q);

    moveit::planning_interface::MoveGroupInterface group_arm_torso("arm_torso");
    // preferred planner
    group_arm_torso.setPlannerId("SBLkConfigDefault");
    ////////////////////
    // PLACE MOVEMENT //
    ////////////////////
    group_arm_torso.setPoseReferenceFrame("base_footprint");
    group_arm_torso.setPoseTarget(place_pose);
    ROS_INFO_STREAM("Planning to move " <<
                    group_arm_torso.getEndEffectorLink() << " to a target pose expressed in " <<
                    group_arm_torso.getPlanningFrame());
    group_arm_torso.setStartStateToCurrentState();
    group_arm_torso.setMaxVelocityScalingFactor(1.0);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    //set maximum time to find a plan
    group_arm_torso.setPlanningTime(8.0);
    bool success = bool(group_arm_torso.plan(my_plan));
    if ( !success ) {
        delete_objects(planning_scene_interface);
        throw std::runtime_error("No plan found for place");
    }
    ROS_INFO_STREAM("Plan found for place in " << my_plan.planning_time_ << " seconds");
    // execute the plan
    ros::WallDuration(1.0).sleep();
    group_arm_torso.move();
    // open gripper
    ros::WallDuration(1.0).sleep();
    // detach the objects using gazebo_ros_link_attacher service
    ros::NodeHandle nh;
    ros::ServiceClient detach_client = nh.serviceClient<gazebo_ros_link_attacher::Attach>("/link_attacher_node/detach");
    // wait for service to be available
    detach_client.waitForExistence();
    gazebo_ros_link_attacher::Attach detach_srv;
    detach_srv.request.model_name_1 = "tiago";
    // detaching links
    detach_srv.request.link_name_1 = "arm_7_link";
    ROS_INFO_STREAM("Detaching object " << req.id);
    if(req.id==1) {
        detach_srv.request.model_name_2 = "Hexagon";
        detach_srv.request.link_name_2 = "Hexagon_link";
    }
    else if(req.id==2) {
        detach_srv.request.model_name_2 = "Triangle";
        detach_srv.request.link_name_2 = "Triangle_link";
    }
    else if(req.id==3) {
        detach_srv.request.model_name_2 = "cube";
        detach_srv.request.link_name_2 = "cube_link";
    }
    openGripper();
    if(!detach_client.call(detach_srv))
        ROS_ERROR("Failed to call service detach");
    ros::WallDuration(2.0).sleep();
    ////////////////////////
    // SAFE POSE MOVEMENT //
    ////////////////////////
    group_arm_torso.setPoseTarget(safe_pose);
    success = bool(group_arm_torso.plan(my_plan));
    if ( !success ) {
        delete_objects(planning_scene_interface);
        throw std::runtime_error("No plan found for safepose");
    }
    ROS_INFO_STREAM("Plan found for initial in " << my_plan.planning_time_ << " seconds");
    // execute the plan
    ros::WallDuration(1.0).sleep();
    group_arm_torso.move();

    spinner.stop();
    res.success = true;
    // removing previous collision objects
    delete_objects(planning_scene_interface);
	return true;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "arm_controller");
    ROS_INFO("Ready to pick and place");

    ros::NodeHandle n_pi;
    ros::NodeHandle n_pl;

    ros::ServiceServer pick_srv = n_pi.advertiseService("pick", pick);
    ros::ServiceServer place_srv = n_pl.advertiseService("place", place);

    spin();

    return EXIT_SUCCESS;
}