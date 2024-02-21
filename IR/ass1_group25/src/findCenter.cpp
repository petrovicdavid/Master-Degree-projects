#include "ros/ros.h"
#include "ass1_group25/findCenter.h"
#include <ass1_group25/positions.h>
#include <ass1_group25/line.h>

#define MAX_RANGE 25
/**
 * Return the middle point of the segment that connects the input points
*/
ass1_group25::positions midpoint(ass1_group25::positions A, ass1_group25::positions B) {
    ass1_group25::positions mid;
    mid.x = (A.x + B.x) / 2;
    mid.y = (A.y + B.y) / 2;
    return mid;
}

/**
 * Return a perpendicolar line.
 * The line is perpendicular with respect to the input line and passing the point in input
*/
ass1_group25::line perpendicularBisector(ass1_group25::line AB, ass1_group25::positions midpoint) {
    ass1_group25::line perpendicular;
    double slope = (AB.end.y - AB.start.y) / (AB.end.x - AB.start.x);

    // Calculate the slope of the perpendicular line
    double perpendicularSlope = -1 / slope;

    // The midpoint becomes the center point of the perpendicular line
    ass1_group25::positions center = midpoint;

    // Find the end point of the perpendicular line using the center point and slope
    perpendicular.start.x = center.x - 10; // Let's assume an arbitrary point on the line
    perpendicular.start.y = center.y - 10 * perpendicularSlope; // Using the equation of the line

    perpendicular.end.x = center.x + 10; // Let's take another point on the line to draw it
    perpendicular.end.y = center.y + 10 * perpendicularSlope;

    return perpendicular;
}

/**
 * Return the position of the point where the two lines intersects
*/
ass1_group25::positions intersection(ass1_group25::line line1, ass1_group25::line line2) {
    double x1 = line1.start.x, y1 = line1.start.y;
    double x2 = line1.end.x, y2 = line1.end.y;
    double x3 = line2.start.x, y3 = line2.start.y;
    double x4 = line2.end.x, y4 = line2.end.y;

    double determinant = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);

    if (determinant == 0) {
        // Perpendicular lines! No intersection!
        // return {0, 0}, so the position of the robot itself
        ass1_group25::positions p;
        p.x = 0, p.y = 0;
        return p;
    }

    double intersectX = ((x1 * y2 - y1 * x2) * (x3 - x4) - (x1 - x2) * (x3 * y4 - y3 * x4)) / determinant;
    double intersectY = ((x1 * y2 - y1 * x2) * (y3 - y4) - (y1 - y2) * (x3 * y4 - y3 * x4)) / determinant;

    //return {intersectX, intersectY};
    ass1_group25::positions res;
    res.x = intersectX, res.y = intersectY;
    return res;
}

/**
 * Manage the requested service
*/
bool manage(ass1_group25::findCenter::Request &req, ass1_group25::findCenter::Response &res) {
	ass1_group25::line AB, CD;
	AB.start = req.A;
	AB.end = req.B;
	CD.start = req.C;
	CD.end = req.D;
	
	// middle points
    ass1_group25::positions midAB = midpoint(AB.start, AB.end);
    ass1_group25::positions midCD = midpoint(CD.start, CD.end);

    // perpendicular line passing in the middle point
    ass1_group25::line perpAB = perpendicularBisector(AB, midAB);
    ass1_group25::line perpCD = perpendicularBisector(CD, midCD);

    // intersection between the perpendicular rects
    res.center = intersection(perpAB, perpCD);
    if(res.center.x==0 && res.center.y==0)
        return false;
    if(res.center.x>=MAX_RANGE || res.center.y>=MAX_RANGE)
        return false;
	return true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "findCenter");
	ros::NodeHandle n;
	ros::ServiceServer service = n.advertiseService("findCenter", manage);
	ROS_INFO("Ready to find the center");
	ros::spin();
	
	return 0;
}
