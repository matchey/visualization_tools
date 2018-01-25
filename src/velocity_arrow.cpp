
//
// src: velocity_arrow.cpp
//
// last update: '18.1.26
// author: matchey
//
// memo:
//   Rviz表示用 Point(x, y, z)から速度ベクトルを表示
//

#include <tf/transform_broadcaster.h>
#include "visualization_tools/velocity_arrow.h"

using namespace std;

int VelocityArrow::id = 0;

VelocityArrow::VelocityArrow()
	:topic_name("/velocity_arrow")
{
	pub = n.advertise<visualization_msgs::Marker>(topic_name, 1);

	va.header.frame_id = "/map";
	va.ns = "/velocity_arrow";
	va.id = id++;

	va.type = visualization_msgs::Marker::ARROW;
	va.action = visualization_msgs::Marker::ADD;

	va.scale.y = 0.1; //width
	va.scale.z = 0.1; //height

	va.color.r = 0.0 + 0.2*(id%6);
	va.color.g = 0.04 + 0.16*(id%7);
	va.color.b = 0.50 + 0.5*(id%2);
	// va.color.b = 0.1 + 0.1*(id%10);
	va.color.a = 0.90;

	va.lifetime = ros::Duration(1.0);

	va.scale.x = 1.0; //length
	va.pose.position.x = 0.0;
	va.pose.position.y = 0.0;
	va.pose.position.z = 0.0;
	va.pose.orientation.w = 1.0;
}

void VelocityArrow::setTopicName(const string& str)
{
	topic_name = str;
	pub = n.advertise<visualization_msgs::Marker>(topic_name, 1);
}

template<class T_p>
void VelocityArrow::setPoint(const T_p& pose)
{
	geometry_msgs::Point p = pose.position;
	v = diff.set(p);
	va.scale.x = diff.norm();
	va.pose.position.x = pose.position.x;
	va.pose.position.y = pose.position.y;
	va.pose.position.z = pose.position.z;
	va.pose.orientation = pose.orientation;
}
template void VelocityArrow::setPoint<geometry_msgs::Pose>(const geometry_msgs::Pose&);

template<class T_p>
void VelocityArrow::setPoint2d(const T_p& p)
{
	v.block<2, 1>(0, 0) = diff.set(p.x, p.y);
	va.scale.x = diff.norm();
	va.pose.position.x = p.x;
	va.pose.position.y = p.y;
	va.pose.position.z = 0.0;
	geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromYaw(atan2(v(1), v(0)));
	va.pose.orientation = quat;
}
template void VelocityArrow::setPoint2d<geometry_msgs::Point>(const geometry_msgs::Point&);

visualization_msgs::Marker VelocityArrow::get() const
{
	return va;
}

void VelocityArrow::publish()
{
	va.header.stamp = ros::Time::now();
	pub.publish(va);
}

ostream& operator << (ostream &os, const VelocityArrow &va)
{
	os << va.va;

	return os;
}

//////////////// pritvate /////////////

