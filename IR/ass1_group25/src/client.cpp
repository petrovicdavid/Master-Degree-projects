#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <ass1_group25/actionAction.h>
#include <ass1_group25/pose.h>
#include <ass1_group25/positions.h>

/**
 * Print the result received from the server
*/
void doneCallback(const actionlib::SimpleClientGoalState& state, const ass1_group25::actionResultConstPtr& result) {
    ROS_INFO("Results (from right to left):");
    int i=0;
    for (ass1_group25::positions position : result->pos) {
        ROS_INFO("Position %d: x=%f, y=%f", i, position.x, position.y);
        i++;
    }
    ROS_INFO("*****************************");
    
}

void activeCallback() { ROS_INFO("Action ready"); }

/**
 * Print feedback received from the server
*/
void feedbackCallback(const ass1_group25::actionFeedbackConstPtr& feedback) { ROS_INFO("Feedback from the server: %s", feedback->f.c_str()); }


int main (int argc, char **argv) {
    ros::init(argc, argv, "client");
    if (argc!=4) {
        ROS_INFO("usage: send navigation goal X Y THETA (degrees)");
        return 1;
    }
    float x = std::stof(argv[1]);
    float y = std::stof(argv[2]);
    float th = std::stof(argv[3]);

    
    // connection to the server
    // send goal
    // wait for result
    
    actionlib::SimpleActionClient<ass1_group25::actionAction> ac("m_thiago", true);
    ROS_INFO("Waiting for the robot.");
    ac.waitForServer(); //will wait for infinite time
    ass1_group25::actionGoal goal;
    ass1_group25::pose pose_msg;
    pose_msg.x=x;
    pose_msg.y=y;
    pose_msg.theta=th;
    goal.M=pose_msg;

    ac.sendGoal(goal, &doneCallback, &activeCallback, &feedbackCallback);

    ac.waitForResult();

    return 0;
}