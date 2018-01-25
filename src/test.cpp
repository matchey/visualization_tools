
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>
#include <pcl_conversions/pcl_conversions.h>
// #include "visualization_tools/bounding_box.h"
#include "visualization_tools/bounding_box_array.h"

using namespace std;

geometry_msgs::Pose pose;
pcl::PointCloud<pcl::PointXYZ>::Ptr pc(new pcl::PointCloud<pcl::PointXYZ>);

void wheelCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
	geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromYaw(msg->pose.pose.orientation.z);
	pose.position = msg->pose.pose.position;
	pose.orientation = quat;
}

void humanCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
	pcl::fromROSMsg(*msg, *pc);
}

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "test_boundig_box");
	cout << "show Bounding Box for Rviz" << endl;

	ros::NodeHandle n;

	ros::Subscriber sub_wheel = n.subscribe<nav_msgs::Odometry>("/tinypower/odom", 1, wheelCallback);
	ros::Subscriber sub_human = n.subscribe<sensor_msgs::PointCloud2>("/human_recognition/positive_pt", 1, humanCallback);

	ros::Publisher pub_human = n.advertise<sensor_msgs::PointCloud2>("/human_recognition/positive_position", 1);

	ros::Time::init();
	ros::Rate loop_rate(10);

	BoundingBox bb;
	BoundingBoxArray bbs;

	pcl::PointCloud<pcl::PointXYZ>::Ptr pc(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointXYZ p;

	while(ros::ok()){
		p.x = pose.position.x;
		p.y = pose.position.y;
		pc->points.push_back(p);
		// pub_human();
		// bb.pc2bb(pc);
		bb.setPose(pose);
		bb.publish();

		bbs.clear();
		// bbs.push_back(bb);
		bbs.push_back(pc);
		// bbs.push_back<pcl::PointCloud<pcl::PointXYZ>::Ptr&>(pc);
		bbs.publish();

		// cout << bb << endl;
		cout << bbs << endl;

		ros::spinOnce();
		loop_rate.sleep();
	}   

	return 0;
}

