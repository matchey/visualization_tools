
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>
#include "visualization_tools/velocity_arrow_array.h"

using namespace std;

class ArrowsPublisher
{
	ros::NodeHandle n;
	ros::Subscriber sub1;
	ros::Subscriber sub2;
		
	VelocityArrowArray vaa;
	geometry_msgs::Pose pose1;
	geometry_msgs::Pose pose2;
	geometry_msgs::Pose pose3;
	geometry_msgs::Pose pose4;

	geometry_msgs::Point p1;
	geometry_msgs::Point p2;

	bool flag_cb1;
	bool flag_cb2;

	template<class T1, class T2>
	void pointSubstitution(T1&, const T2&);
	template<class T1, class T2>
	void quatSubstitution(T1&, const T2&);

	public:
	ArrowsPublisher()
		: flag_cb1(false), flag_cb2(false)
	{
		// sub1 = n.subscribe<geometry_msgs::TransformStamped>("/vicon/aotn1/aotn1", 1,
		// 		&ArrowsPublisher::Point1Callback, this);
		// sub2 = n.subscribe<geometry_msgs::TransformStamped>("/vicon/aotn2/aotn2", 1,
		// 		&ArrowsPublisher::Point2Callback, this);
		sub1 = n.subscribe<geometry_msgs::TransformStamped>("/vicon/infant_velodyne/infant_velodyne", 1,
				&ArrowsPublisher::Point1Callback, this);
		sub2 = n.subscribe<geometry_msgs::TransformStamped>("/vicon/person_1/person_1", 1,
				&ArrowsPublisher::Point2Callback, this);

		vaa.filter(20); // 何点分使って平均出すか [100Hzでviconでてるから5だと20Hzで全更新]
	}

	void Point1Callback(const geometry_msgs::TransformStamped::ConstPtr& msg){
		pointSubstitution(p1, msg->transform.translation);
		pointSubstitution(pose1.position, msg->transform.translation);
		quatSubstitution(pose1.orientation, msg->transform.rotation);
		flag_cb1 = true;
	}
	void Point2Callback(const geometry_msgs::TransformStamped::ConstPtr& msg){
		pointSubstitution(p2, msg->transform.translation);
		pointSubstitution(pose2.position, msg->transform.translation);
		quatSubstitution(pose2.orientation, msg->transform.rotation);
		flag_cb2 = true;
	}

	bool isSet();
	void pub_arrows();
};

template<class T1, class T2>
void ArrowsPublisher::pointSubstitution(T1& p1, const T2& p2)
{
	p1.x = p2.x;
	p1.y = p2.y;
	p1.z = p2.z;
}

template<class T1, class T2>
void ArrowsPublisher::quatSubstitution(T1& q1, const T2& q2)
{
	q1.x = q2.x;
	q1.y = q2.y;
	q1.z = q2.z;
	q1.w = q2.w;
}

bool ArrowsPublisher::isSet()
{
	bool rtn = flag_cb1 && flag_cb2;

	if(rtn) flag_cb1 = flag_cb2 = false;
			
	return rtn;
}

void ArrowsPublisher::pub_arrows()
{
	pose3.position.x = pose1.position.x + pose1.orientation.x*pose2.position.x;
	pose3.position.y = pose1.position.y + pose2.position.y;
	pose3.position.z = pose1.orientation.z*pose1.position.z + pose2.position.z;
	quatSubstitution(pose3.orientation, pose1.orientation);
	pose3.orientation.w = pose1.orientation.w + pose2.orientation.w;

	pose4.position.x = pose1.position.x - pose2.position.x;
	pose4.position.y = pose1.position.y - pose1.orientation.y*pose2.position.y;
	pose4.position.z = pose1.position.z - pose2.position.z;
	quatSubstitution(pose4.orientation, pose2.orientation);
	pose4.orientation.w = pose1.orientation.w - pose2.orientation.w;

	// vaa.setPoints(p1, p2);
	vaa.setPoints(pose1, pose2);
	// vaa.setPoints(pose1, pose2, pose3 ,pose4, p1, p2);
	vaa.publish();
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "test_arrow");
	cout << "TransformStampeds to Arrows" << endl;

	ArrowsPublisher ap;

	ros::Rate loop_rate(100);

	while(ros::ok()){
		if(ap.isSet()) ap.pub_arrows();
		// ap.pub_arrows();

		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}

