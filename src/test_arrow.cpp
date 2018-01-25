
#include <ros/ros.h>
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
	geometry_msgs::Pose p1;
	geometry_msgs::Pose p2;

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
		sub1 = n.subscribe<geometry_msgs::TransformStamped>("/vicon/aotn1/aotn1", 1,
				&ArrowsPublisher::Point1Callback, this);
		sub2 = n.subscribe<geometry_msgs::TransformStamped>("/vicon/aotn2/aotn2", 1,
				&ArrowsPublisher::Point2Callback, this);
	}

	void Point1Callback(const geometry_msgs::TransformStamped::ConstPtr& msg){
		pointSubstitution(p1.position, msg->transform.translation);
		pointSubstitution(p1.orientation, msg->transform.rotation);
		flag_cb1 = true;
	}
	void Point2Callback(const geometry_msgs::TransformStamped::ConstPtr& msg){
		pointSubstitution(p2.position, msg->transform.translation);
		pointSubstitution(p2.orientation, msg->transform.rotation);
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
	vaa.setPoints(p1, p2);
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

