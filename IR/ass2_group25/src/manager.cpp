#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <ass1_group25/actionAction.h>
#include <ass1_group25/pose.h>
#include <ass1_group25/positions.h>
#include <tiago_iaslab_simulation/Objs.h>
#include <vector>
#include <control_msgs/PointHeadAction.h>
#include <ass2_group25/detection.h>
#include <ass2_group25/objects.h>
#include <ass2_group25/pick.h>
#include <ass2_group25/place.h>
#include <map>
#include <string>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "ass2_group25/detector_colored_tables.h"
#include "ass1_group25/positions.h"

using namespace std;

// structure to save the id and the pose of the object to pick
struct obj_pick_position {
	int id;
	ass1_group25::pose p;
};
// structure to save the id and the position of the table
struct colored_table_position {
	int id;
	ass1_group25::positions p;
};
// vector of colored_table_position objects
vector<colored_table_position> colored_tables;
bool first_time = true;

// move the head of the robot
void move_head(float z){
	actionlib::SimpleActionClient<control_msgs::PointHeadAction> hc("head_controller/point_head_action", true);
	hc.waitForServer();
	control_msgs::PointHeadGoal headGoal;
	
	// the goal consists in making the Z axis of the cameraFrame to point towards the pointStamped
	headGoal.pointing_frame = "/xtion_rgb_optical_frame";
	headGoal.min_duration = ros::Duration(0.5);
	headGoal.max_velocity = 1;
	headGoal.target.header.frame_id = "base_footprint";
	headGoal.target.point.x = 1.0;
	headGoal.target.point.y = 0.0;
	headGoal.target.point.z = z; 

	hc.sendGoal(headGoal);
	ros::Duration(0.5).sleep();
}

// move the robot to a position
void move_tiago(obj_pick_position pos){
	actionlib::SimpleActionClient<ass1_group25::actionAction> ac("m_thiago", true);
	
	ac.waitForServer();
	ass1_group25::actionGoal goal;
	ass1_group25::pose pose_msg;
	
	// Move to waypoint
	pose_msg.x = pos.p.x;
	pose_msg.y = pos.p.y;
	pose_msg.theta = pos.p.theta;
	goal.M = pose_msg;

	ac.sendGoal(goal);

	ac.waitForResult();
}

// callback for the action server to get the results
void doneCallback(const actionlib::SimpleClientGoalState& state, const ass1_group25::actionResultConstPtr& result) {
    ROS_INFO("Results (from right to left):");
    int i=0;
    for (ass1_group25::positions position : result->pos) {
		if(first_time){
			colored_tables[i].p.x = position.x;
			colored_tables[i].p.y = position.y;
		} else {
			int d = colored_tables.size();
			
			colored_tables[d-i-1].p.x += position.x;
			colored_tables[d-i-1].p.y += position.y;
			colored_tables[d-i-1].p.x = colored_tables[d-i-1].p.x / 2;
			colored_tables[d-i-1].p.y = colored_tables[d-i-1].p.y / 2;
			ROS_INFO("Position %d: x=%f, y=%f", i, position.x, position.y);
			ROS_INFO("*****************************");
		}
        i++;
    }
    first_time = false;
}

// move the robot to a position and get the results if wanted
void move_tiago(float x, float y, float th, bool req_result=false){
	actionlib::SimpleActionClient<ass1_group25::actionAction> ac("m_thiago", true);
    
    ac.waitForServer();
    ass1_group25::actionGoal goal;
    ass1_group25::pose pose_msg;
    
    // Move to waypoint
    pose_msg.x=x;
	pose_msg.y=y;
	pose_msg.theta=th;
	goal.M=pose_msg;

	if(req_result==false)
		ac.sendGoal(goal);
	else
		ac.sendGoal(goal, &doneCallback);

	ac.waitForResult();
}

// set position of the robot in the map
obj_pick_position set_position(int id, float x, float y, float th){
	obj_pick_position pos;
	pos.id = id;
	pos.p.x = x;
	pos.p.y = y;
	pos.p.theta = th;
	return pos;
}

int main (int argc, char **argv) {
    ros::init(argc, argv, "manager");

	// map of (string, obj_pick_position) objects
	map<string, obj_pick_position> positions;
    
    // save the positions from which pick the objects
    vector<obj_pick_position> objs;
    obj_pick_position blue, green, red;

	blue = set_position(1, 8.079, -1.869, -90);
    objs.push_back(blue);
    
	green = set_position(2, 7.679, -4.069, 90);
    objs.push_back(green);
    
	red = set_position(3, 7.579, -1.869, -90);
    objs.push_back(red);

	//////////////////////////////////
	// WAYPOINTS FOR ROBOT MOVEMENT //
	//////////////////////////////////
	positions["NW"] = set_position(-1, 9.079, -4.369, 135);		// up left table
	positions["NW_L"] = set_position(-1, 9.079, -4.369, 0);		// up left table looking left
	positions["SW"] = set_position(-1, 9.079, -1.869, -135);	// down left table
	positions["SE"] = set_position(-1, 7.079, -1.869, -45);		// down right table    
    
    // get the order in which we have to pick the objects
    vector<int> indices;
    ros::NodeHandle nd;
	ros::ServiceClient client = nd.serviceClient<tiago_iaslab_simulation::Objs>("human_objects_srv");
	tiago_iaslab_simulation::Objs srv;
	srv.request.ready = true;
	srv.request.all_objs = true;
	if(client.call(srv)) {
		indices = srv.response.ids;
	} else {
		ROS_ERROR("Failed to call human_objects_srv");
	}	
	
	// order the objects in our vector
	sort(objs.begin(), objs.end(), [&indices](const obj_pick_position& a, const obj_pick_position& b) {
        return find(indices.begin(), indices.end(), a.id) < find(indices.begin(), indices.end(), b.id);
    });
    
    // to see if the order is correct
	ROS_INFO("Order of objects to pick:");
    for(int i=0;i<indices.size();i++){
		ROS_INFO("%d", objs[i].id);
    }
    
	// initial waypoint
	move_tiago(positions["SW"]);
	// pose between two rooms
	move_tiago(positions["NW_L"]);
    // pose to detect the tables' colors
	move_tiago(10.5, -2.3, 60);
	
    ros::NodeHandle nt;
	ros::ServiceClient client_color_tables = nt.serviceClient<ass2_group25::detector_colored_tables>("colored_tables_detection");
	ass2_group25::detector_colored_tables srv_det_col;
	
	srv_det_col.request.H.stamp = ros::Time::now();
	srv_det_col.request.ready = true;
	
	if(client_color_tables.call(srv_det_col)) {
		ROS_INFO("Order in which tables appears in front of the robot (from right to left):");
		for(int j=0;j<srv_det_col.response.ids.size();j++){
			ROS_INFO("Table id: %d", srv_det_col.response.ids[j]);
			colored_table_position table;
			table.id = srv_det_col.response.ids[j];
			colored_tables.push_back(table);
		}
	}
	else {
		ROS_ERROR("Failed to find detection node");
	}
	// moving tiago and saving the position of the tables
    move_tiago(10.5, -2.3, 90, true);
	// waypoint between wall on the left and cylindrical table on the left
    move_tiago(13.3, -1, 90);
	
	//ROS_INFO("Move to pos 2 and laserScan detection");
	// moving tiago and saving the position of the tables
	move_tiago(11.2, 1.7, -35, true);
	// ordering colored_tables vector in a way that in position 0 there is the table of the first object to pick and in position 2 the last
    sort(colored_tables.begin(), colored_tables.end(), [&indices](const colored_table_position& a, const colored_table_position& b) {
        return find(indices.begin(), indices.end(), a.id) < find(indices.begin(), indices.end(), b.id);
    });
    
    // printing to ensure the order is correct
    for(int i=0;i<colored_tables.size();i++){
		//ROS_INFO("id: %d  x = %f y = %f", colored_tables[i].id, colored_tables[i].p.x, colored_tables[i].p.y);
    }
	// moving again to the initial waypoint
	move_tiago(positions["SW"]);
    
    // start pick-and-place procedure for each object
    for(int i=0;i<objs.size();i++) {
		// check if object to pick is green
		// waypoint for green object
		if(objs[i].id == 2) {
			// if first object or previous was blue, left waypoint
				move_tiago(positions["NW"]);
		}
		// check if object to pick is red
		// waypoint for red object
		else if(objs[i].id == 3) {
			move_tiago(positions["SE"]);
		}
		// check if object to pick is blue
		// waypoint for blue object
		else if(objs[i].id == 1 && i!=0) {
			move_tiago(positions["SW"]);
		}
		// moving to the object
		move_tiago(objs[i]);
		
		ROS_INFO("Arrived to destination in which to pick obj %d", objs[i].id);
		
		move_head(0.7);
		ROS_INFO("Head in position");
		
		//make detection
		ros::NodeHandle nd;
        ros::ServiceClient client_detection = nd.serviceClient<ass2_group25::detection>("detection");
        ass2_group25::detection srv_det;
        
		srv_det.request.H.stamp = ros::Time::now();
        srv_det.request.ready = true;

		ros::Duration(0.5).sleep();
		// printing detected objects on the squared table
        if(client_detection.call(srv_det)) {
            for(int j=0;j<srv_det.response.detection.size();j++){
				ROS_INFO("Object ID: %d (x, y, z): (%f, %f, %f)", 	srv_det.response.detection[j].id, 
																	srv_det.response.detection[j].position.pose.position.x,
																	srv_det.response.detection[j].position.pose.position.y,
																	srv_det.response.detection[j].position.pose.position.z);
			}
        }
        else {
            ROS_ERROR("Failed to find detection node");
        }
		
	    move_head(1);
		ROS_INFO("Head in base position");

		//////////
		// PICK //
		//////////
		ros::NodeHandle n_pick;
        ros::ServiceClient pick_client = n_pick.serviceClient<ass2_group25::pick>("pick");
        ass2_group25::pick srv_pick;
		srv_pick.request.H.stamp = ros::Time::now();
		for(int j=0;j<srv_det.response.detection.size();j++) {
			// send to pick only the object we want to pick and "noisy" objects
			if(srv_det.response.detection[j].id == objs[i].id || srv_det.response.detection[j].id >= 4)
				srv_pick.request.detection.push_back(srv_det.response.detection[j]);
		}
		ros::Duration(0.5).sleep();
		// actually calling the pick service
        if(pick_client.call(srv_pick)) {
            if(srv_pick.response.success)
				ROS_INFO("Pick success");
			else
				ROS_INFO("Pick failed");
        }
        else {
            ROS_ERROR("Failed to find pick node");
        }

		///////////////////////
		// MOVEMENT TO PLACE //
		///////////////////////
		// if blue or red move to SW
		if(objs[i].id == 1 || objs[i].id == 3) {
			move_tiago(positions["SW"]);
			move_tiago(positions["NW_L"]);
		} else if(objs[i].id == 2) {
			move_tiago(positions["NW_L"]);
		}
		// move to place pose
		// in y axis we subtract 0.65 to have the right amout of space between the table and the robot
		move_tiago(colored_tables[i].p.x, colored_tables[i].p.y-0.65, 90);

		///////////
		// PLACE //
		///////////
		ros::NodeHandle n_place;
		ros::ServiceClient place_client = n_place.serviceClient<ass2_group25::place>("place");
		ass2_group25::place srv_place;
		srv_place.request.H.stamp = ros::Time::now();
		srv_place.request.id = objs[i].id;
		srv_place.request.position.x = colored_tables[i].p.x;
		srv_place.request.position.y = colored_tables[i].p.y;
		ros::Duration(0.5).sleep();
		// actually calling the place service
		if(place_client.call(srv_place)) {
			if(srv_place.response.success)
				ROS_INFO("Place success");
			else
				ROS_INFO("Place failed");
		}
		else {
			ROS_ERROR("Failed to find place node");
		}
		
		// if blue go to NW
		if(objs[i].id == 1) {
			move_tiago(positions["NW"]);

			if(i<2 && objs[i+1].id == 3) // if next object is red go also to SW to avoid table
				move_tiago(positions["SW"]);
		}
		// if green go to NW
		else if(objs[i].id == 2) {
			move_tiago(positions["NW"]);

			if(i<2 && objs[i+1].id == 3) // if next object is red go also to SW to avoid table
				move_tiago(positions["SW"]);
		}
		// if red go to NW
		else if(objs[i].id == 3) {
			move_tiago(positions["NW"]);
		}
		ROS_INFO("Come back");
	}
	
    return 0;
}
