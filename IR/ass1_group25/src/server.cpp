#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <ass1_group25/actionAction.h>
#include <ass1_group25/findCenter.h>
#include <sensor_msgs/LaserScan.h>
#include <ass1_group25/pose.h>
#include <ass1_group25/positions.h>


typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

std::vector<ass1_group25::positions> centers;
float transformationMatrix[4][4];
#define THRESHOLD_SEGMENTATION_DERIVATIVE 50
#define WINDOW_SIZE_SMOOTH 1

struct Point {
    float x, y;     // coordinates

    /**
     * Input polar point, save it in cartesian
    */
    Point(float r, float th) : 
        // conversion from polar to cartesian
        x(r*cos(th)), // x = radiuscos(theta)
        y(r*sin(th)) // y = radiussin(theta)
    {}
    
    /**
     * Input cartesian coordiantes and saves it as a cartesian point
    */
    Point(float x_input, float y_input,bool is_cartesian) : 
        // conversion from polar to cartesian
        x(x_input), // x = radiuscos(theta)
        y(y_input) // y = radiussin(theta)
    {}
};

/**
 * Derivative over two polar points
*/
float findiff(float fxh, float fx, float h) {
    return (fxh - fx) / h;
}

/**
 * Derivative function over set of polar points
*/
void calc_first_derivative_polar(std::vector<float> data, std::vector<float> &output, float incrementTheta){
    for(int i = 0; i < data.size()-1; i++) {
        output.push_back(findiff(data[i+1], data[i], incrementTheta));
    }
}

/**
 * Divide data in segments using the derivative
 * Input:
 * - ranges: input data.
 * - f_deriv: first polar derivative.
 * 
 * Output:
 * - segmentedData: vector of vectors of points where the segments are divided and saved.
 * - segment_index_begin: vector of indexes associated with segmentedData.
*/
void divide_in_segment(std::vector<float> ranges,std::vector<float> f_deriv, std::vector<std::vector<float>> &segmentedData, std::vector<int> &segment_index_begin){
    int segment_index = 0;
    double begin_flag = true;

    std::vector<float> segment;
    for(int i = 0; i < ranges.size()-1; i++) {
        // if absolute value of derivative is less than a threshold we are in the same segment
        if(std::fabs(f_deriv[i]) < THRESHOLD_SEGMENTATION_DERIVATIVE){
            // initial position of the segment
            if(begin_flag){
                // saving corresponding index
                segment_index_begin.push_back(i);
                begin_flag = false;
            }
            segment.push_back(ranges[i]);
        }
        else { // new segment has began
            if(segment.size() > 0)
                segmentedData.push_back(segment);
            segment.clear();
            begin_flag = true;
        }
    }
    if(segment.size() > 0)  // last segment
        segmentedData.push_back(segment);
}

/**
 * Conversion from polar to cartesian positions in order to have "normalized" data, so like we see every segment from the same point of view (in front of us).
 * Input:
 * - segmentedData: data in polar form
 * - initial_theta: initial angle for polar data rotation
 * - incrementTheta: radiant angle increment of the robot
 * 
 * Output:
 * - segmentedDataCartesian: data in cartesian form
*/
void segmentPolar_to_segmentCartesian(std::vector<std::vector<float>> segmentedData, std::vector<std::vector<Point>> &segmentedDataCartesian, float initial_theta, float incrementTheta){
    for(int i = 0; i < segmentedData.size(); i++){
        std::vector<Point> segmentCartesian;
        // initial angle in the specific segment i
        float initial_theta_i = initial_theta + (segmentedData[i].size()/2)*incrementTheta;
        for(int j = 0; j < segmentedData[i].size(); j++){
            // creation of the cartesian point
            segmentCartesian.push_back(Point(segmentedData[i][j], initial_theta_i + (j*incrementTheta)));
        }
        // save the segment i in cartesian coordinates
        segmentedDataCartesian.push_back(segmentCartesian);
    }
}

/**
 * Smoothing data segments for removing noise in cartesian space
*/
std::vector<std::vector<Point>> smooth_data_cartesian(std::vector<std::vector<Point>> dataSegmentedPoint, int window_size){
    std::vector<std::vector<Point>> dataSegmentedPoint_smooth;
    for(int i = 0; i < dataSegmentedPoint.size(); i++){
        std::vector<Point> segment_smooth;
        for(int j = 0; j < dataSegmentedPoint[i].size(); j++){
            if (j < window_size || j >= dataSegmentedPoint[i].size() - window_size) {
                // if data is too close to an edge it is not possible to compute the mean
                segment_smooth.push_back(dataSegmentedPoint[i][j]);
            } else {
                // compute mean of data in the range
                double total_y = 0;
                for (int k = j - window_size; k < j + window_size + 1; ++k) {
                    total_y += dataSegmentedPoint[i][k].y;
                }
                float mean = total_y / (window_size * 2 + 1);
                segment_smooth.push_back(Point(dataSegmentedPoint[i][j].x, mean, true));
            }
        }
        dataSegmentedPoint_smooth.push_back(segment_smooth);
    }
    return dataSegmentedPoint_smooth;
}

/**
 * Compute first derivative of smoothed segments
*/
std::vector<std::vector<Point>> calc_first_derivative_segment_smooth(std::vector<std::vector<Point>> data){
    std::vector<std::vector<Point>> first_derivative_segment_smooth;
    for(int i = 0; i < data.size(); i++){
        std::vector<Point> derivative_segment_smooth;
        for(int j = 0; j < data[i].size()-1; j++){
            derivative_segment_smooth.push_back(Point(data[i][j].x,findiff(data[i][j+1].y, data[i][j].y, data[i][j+1].x - data[i][j].x), true));
        }
        first_derivative_segment_smooth.push_back(derivative_segment_smooth);
    }
    return first_derivative_segment_smooth;
}

/**
 * Main function for detection of the moving objects
*/
std::vector<std::vector<Point>> circles_points(std::vector<float> ranges, float incrementTheta, float thetaBegin){

    //calcualte the first derivative of the polar data
    std::vector<float> first_derivative_polar;
    calc_first_derivative_polar(ranges, first_derivative_polar,incrementTheta);

    //divide in segment
    std::vector<std::vector<float>> segmentedData;
    std::vector<int> segment_index_begin;

    divide_in_segment(ranges,first_derivative_polar, segmentedData, segment_index_begin);

    //trasform each segment in cartesian
    std::vector<std::vector<Point>> segmentedDataCartesian;
    float const initial_theta_centered = M_PI/2;
    segmentPolar_to_segmentCartesian(segmentedData, segmentedDataCartesian,initial_theta_centered, incrementTheta);

    //smooth segenment
    std::vector<std::vector<Point>> segmentedDataCartesian_smooth;
    segmentedDataCartesian_smooth = smooth_data_cartesian(segmentedDataCartesian, WINDOW_SIZE_SMOOTH);

    //calcualte the derivative of each segment_smooth
    std::vector<std::vector<Point>> first_derivative_segment_smooth;
    first_derivative_segment_smooth = calc_first_derivative_segment_smooth(segmentedDataCartesian_smooth);

 
    //part below is implemented to find the segment that are parabolas (thus, moving objects)
    
    std::vector<int> segment_filtered_index;
    //check if the first derivative is near to zero
    for(int i = 0; i < first_derivative_segment_smooth.size(); i++) {
        int count = 0;
        for(int j = 0; j < first_derivative_segment_smooth[i].size(); j++) {
            if(std::fabs(first_derivative_segment_smooth[i][j].y) < 0.1) {
                count ++;
            }
        }
        if(count<10 && count > 0) { //the count threshold defined from observation of the data plot
            segment_filtered_index.push_back(i);
        }
    }

    //if the segments are too short, remove them
    std::vector<int> segment_filtered_index_long;
    for(int i = 0; i < segment_filtered_index.size(); i++) {
        if(segmentedDataCartesian_smooth[segment_filtered_index[i]].size() > 6) { //the count threshold defined from observation of the data plot
            segment_filtered_index_long.push_back(segment_filtered_index[i]);
        }
    }

    //if the derivative after the fist zero is positive, add the segment to the list of moving objects
    std::vector<int> segment_filtered_index_positive;
    for(int i = 0; i < segment_filtered_index_long.size(); i++) {
        float total = 0;
        int count = 0;
        for(int j = 0; j < first_derivative_segment_smooth[segment_filtered_index_long[i]].size(); j++) { //cheking from the beginning because the vector is reversed
            if(!(std::fabs(first_derivative_segment_smooth[segment_filtered_index_long[i]][j].y) < 0.1)) {
                total += first_derivative_segment_smooth[segment_filtered_index_long[i]][j].y;
                count ++;
            } else {
                break;
            }
        }
        double mean = total/count;
        if(mean > 0) {
            segment_filtered_index_positive.push_back(segment_filtered_index_long[i]);
        }
    }

    //if the derivative from the end to the fist zero is negative, add to the list
    std::vector<int> segment_filtered_index_negative;
    for(int i = 0; i < segment_filtered_index_positive.size(); i++) {
        float total = 0;
        int count = 0;
        for(int j = first_derivative_segment_smooth[segment_filtered_index_positive[i]].size()-1; j > 0; j--) { //cheking from the beginning because the vector is reversed
            if(!(std::fabs(first_derivative_segment_smooth[segment_filtered_index_positive[i]][j].y) < 0.1)) {
                total += first_derivative_segment_smooth[segment_filtered_index_positive[i]][j].y;
                count ++;
            } else {
                break;
            }
        }
        double mean = total/count;
        if(mean < 0){
            segment_filtered_index_negative.push_back(segment_filtered_index_positive[i]);
        }
    }

    std::vector<int> index_circle_segment = segment_filtered_index_negative;

    //transform index referred to the segment index to the index referred to the original data
    std::vector<int> index_circle_segment_original,lengthSegment_circle_segment_original;
    for(int i = 0; i < index_circle_segment.size(); i++) {
        index_circle_segment_original.push_back(segment_index_begin[index_circle_segment[i]]);
        lengthSegment_circle_segment_original.push_back(segmentedData[index_circle_segment[i]].size());
    }


    //create polar vectors with circle segments
    std::vector<std::vector<float>> circle_segment_polar;
    for(int i = 0; i < index_circle_segment_original.size(); i++) {
        std::vector<float> segment;
        for(int j = 0; j < lengthSegment_circle_segment_original[i]; j++) {
            segment.push_back(ranges[index_circle_segment_original[i]+j]);
        }
        circle_segment_polar.push_back(segment);
    }

    //polar to cartesian segment with the correct reference
    std::vector<std::vector<Point>> circle_segment_cartesian;
    for(int i = 0; i < circle_segment_polar.size(); i++) {
        std::vector<Point> segment;
        float theta = thetaBegin + index_circle_segment_original[i]*incrementTheta;
        for(int j = 0; j < circle_segment_polar[i].size(); j++) {
            segment.push_back(Point(circle_segment_polar[i][j], theta + (j*incrementTheta)));
        }
        circle_segment_cartesian.push_back(segment);
    }
    return circle_segment_cartesian;
}

void data(const sensor_msgs::LaserScan::ConstPtr& pos) {
    std::vector<Point> points; // cartesian points storage
    std::vector<float> polar_dist{pos->ranges.begin()+20,pos->ranges.end()-20}; // remove the feet of the robot from scan
    
    std::vector<std::vector<Point>> circles_points_vec = circles_points(polar_dist, pos->angle_increment, pos->angle_min);    

    for(int i=0; i<circles_points_vec.size(); i++) {
        ros::NodeHandle nd;
        ros::ServiceClient client = nd.serviceClient<ass1_group25::findCenter>("findCenter");
        ass1_group25::findCenter srv;
        ass1_group25::positions A, B, C, D; // points used to find center
        
        // points assignment
        std::vector<Point> actual_seg = circles_points_vec.at(i);
        int count = actual_seg.size(); // number of elements
        
        A.x = actual_seg[0].x;
        A.y = actual_seg[0].y;

        B.x = actual_seg[((int)count/2)+1].x;
        B.y = actual_seg[((int)count/2)+1].y;

        C.x = actual_seg[((int)count/2)-1].x;
        C.y = actual_seg[((int)count/2)-1].y;

        D.x = actual_seg[count-1].x;
        D.y = actual_seg[count-1].y;

        srv.request.A = A;
        srv.request.B = B;
        srv.request.C = C;
        srv.request.D = D;

        if(client.call(srv)) {  // call findCenter
            centers.push_back(srv.response.center);
        }
        else {
            ROS_ERROR("Failed to find center for object %d", (i+1));
        }
    }
}

void position(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
    float x = msg->pose.pose.position.x;
    float y = msg->pose.pose.position.y;
    // z not needed
    geometry_msgs::Quaternion orientation = msg->pose.pose.orientation;
    tf2::Quaternion quat(orientation.x, orientation.y, orientation.z, orientation.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    transformationMatrix[0][0] = cos(yaw);transformationMatrix[0][1] = -sin(yaw);transformationMatrix[0][2] = 0.0;transformationMatrix[0][3] = x;
    transformationMatrix[1][0] = sin(yaw);transformationMatrix[1][1] = cos(yaw);transformationMatrix[0][2] = 0.0;transformationMatrix[1][3] = y;
    transformationMatrix[2][0] = 0.0;transformationMatrix[2][1] = 0.0;transformationMatrix[2][2] = 1.0;transformationMatrix[2][3] = 0.0;
    transformationMatrix[3][0] = 0.0;transformationMatrix[3][1] = 0.0;transformationMatrix[3][2] = 0.0;transformationMatrix[3][3] = 1.0;
}

class actionAction {
    protected:
        ros::NodeHandle nh_;
        actionlib::SimpleActionServer<ass1_group25::actionAction> as_;
        std::string action_name_;
        ass1_group25::actionFeedback feedback_;
        ass1_group25::actionResult result_;
    public:
        actionAction(std::string name) : as_(nh_, name, boost::bind(&actionAction::executeCB, this, _1), false), action_name_(name) {
            as_.start();
        }
        ~actionAction(void){}
        void executeCB(const ass1_group25::actionGoalConstPtr &goal) {
            bool success = true;
            result_.pos.clear();
			
            feedback_.f = "Processing the request...";
            as_.publishFeedback(feedback_);
            
            //tell the action client that we want to spin a thread by default
            MoveBaseClient ac("move_base", true);

            //wait for the action server to come up
            while(!ac.waitForServer(ros::Duration(5.0))) {
                feedback_.f = "Waiting for the move_base action server to come up";
                as_.publishFeedback(feedback_);
            }
            // goal definition
            move_base_msgs::MoveBaseGoal mb_goal;

            mb_goal.target_pose.header.frame_id = "map";
            mb_goal.target_pose.header.stamp = ros::Time::now();

            mb_goal.target_pose.pose.position.x = goal->M.x;
            mb_goal.target_pose.pose.position.y = goal->M.y;
            // from degrees to radiants to quaternions
            double angle_in_degrees = goal->M.theta;
            double angle_in_radians = angle_in_degrees*(M_PI/180);
            tf2::Quaternion quaternion;
            quaternion.setRPY(0, 0, angle_in_radians);
            quaternion.normalize();
            mb_goal.target_pose.pose.orientation = tf2::toMsg(quaternion);
            // sending goal to move_base server
            feedback_.f = "Sending goal to move_base server";
            as_.publishFeedback(feedback_);
            ac.sendGoal(mb_goal);

            ac.waitForResult();
            // goal reached
            if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
                feedback_.f = "Goal reached!";
                as_.publishFeedback(feedback_);
                
                // subscribe to laser's topic and wait for data
                feedback_.f = "Start detection...";
                as_.publishFeedback(feedback_);
				ros::NodeHandle n;
				ros::Subscriber sub = n.subscribe("scan", 1000, data);
				
				sensor_msgs::LaserScan::ConstPtr msg_scan = ros::topic::waitForMessage<sensor_msgs::LaserScan>("scan", ros::Duration(10.0));			
				// controls if detection is done
				if(msg_scan){
                    feedback_.f = "Detection finished";
					as_.publishFeedback(feedback_);
				} else {
					feedback_.f = "Error in detection!";
					as_.publishFeedback(feedback_);
					success = false;
				}
            }
            else {
                feedback_.f = "Goal not reached! Check input parameters!";
                as_.publishFeedback(feedback_);
                success = false;
            }
            
            // from relative coordinates of the centers to absolute ones
            ros::NodeHandle nc;
            ros::Subscriber coord = nc.subscribe("robot_pose", 1000, position);
            geometry_msgs::PoseWithCovarianceStamped::ConstPtr msg_coord = ros::topic::waitForMessage<geometry_msgs::PoseWithCovarianceStamped>("robot_pose", ros::Duration(2.0));

            if(success) {
                if(msg_coord) {
                    for(int i=0; i<centers.size(); i++) {
                        ass1_group25::positions p;
                        p.x = transformationMatrix[0][0] * centers[i].x + transformationMatrix[0][1] * centers[i].y + transformationMatrix[0][3];
                        p.y = transformationMatrix[1][0] * centers[i].x + transformationMatrix[1][1] * centers[i].y + transformationMatrix[1][3];
                        
                        result_.pos.push_back(p);
                    }
                } else {
                    for(int i=0; i<centers.size(); i++)
                        result_.pos.push_back(centers[i]);
                }
                ROS_INFO("%s: Succeeded", action_name_.c_str());
                as_.setSucceeded(result_);
            }
            centers.clear();
        }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "server");
    actionAction m_thiago("m_thiago");
    ros::spin();

    return 0;
}