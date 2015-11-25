#include <ros/ros.h>
#include "std_msgs/String.h"
#include <sensor_msgs/LaserScan.h>
#include <stdio.h>

using namespace std;
class Scan2{
public:
  Scan2();
private:
  ros::NodeHandle n;
    ros::Publisher scan_pub;
    ros::Subscriber scan_sub;
  void scanCallBack(const sensor_msgs::LaserScan::ConstPtr& scan2);
};

Scan2::Scan2()
{
	scan_pub = n.advertise<sensor_msgs::LaserScan>("/scan2",1);
	scan_sub = n.subscribe<sensor_msgs::LaserScan>("/scan",1, &Scan2::scanCallBack, this);
}

void Scan2::scanCallBack(const sensor_msgs::LaserScan::ConstPtr& scan2)
{
	int ranges = scan2->ranges.size();
	//populate the LaserScan message
	sensor_msgs::LaserScan scan;
    	scan.header.stamp = scan2->header.stamp;
    	scan.header.frame_id = scan2->header.frame_id;
    	scan.angle_min = scan2->angle_min;
    	scan.angle_max = scan2->angle_max;
    	scan.angle_increment = scan2->angle_increment;
    	scan.time_increment = scan2->time_increment;
	scan.range_min = 0.0;
	scan.range_max = 100.0;    
	scan.ranges.resize(ranges);
	for(int i = 0; i < ranges; ++i)
	{
    		scan.ranges[i] = scan2->ranges[i] + 1;
	}
	scan_pub.publish(scan);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "example2_laser_scan_publisher");
	Scan2 scan2;
	ros::spin();
}
