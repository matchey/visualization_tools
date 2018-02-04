
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
	:topic_name("/velocity_arrow"),
	 save(1), cnt(0), filled(false), velocity_sum(0.0),
	 flag_pub(false)
{
	initialize();
}

void VelocityArrow::setTopicName(const string& str)
{
	topic_name = str;
	pub = n.advertise<visualization_msgs::Marker>(topic_name, 1);
}

void VelocityArrow::setFrameID(const string& str)
{
	va.header.frame_id = str;
}

void VelocityArrow::setPoint(const geometry_msgs::Pose& pose)
{
	geometry_msgs::Point p = pose.position;
	v = diff.set(p);
	setNorm();
	va.pose.position.x = pose.position.x;
	va.pose.position.y = pose.position.y;
	va.pose.position.z = pose.position.z;
	va.pose.orientation = pose.orientation;
	va.header.stamp = ros::Time::now();
}

template<class T_p>
void VelocityArrow::setPoint(const T_p& p)
{
	va.pose.position.x = p.x;
	va.pose.position.y = p.y;
	va.pose.position.z = p.z;
	// v = diff.set(p).normalized();
	v = diff.set(p);
	// va.scale.x = diff.norm();
	setNorm();
	Eigen::Vector3d vn = Eigen::Vector3d(v(0)/diff.norm() + 1,
			                             v(1)/diff.norm(),
										 v(2)/diff.norm()).normalized();
	// Eigen::Vector3d vn = Eigen::Vector3d(v(0)+1, v(1), v(2)).normalized();
	if(fabs(va.scale.x) > 0.09){ // [1km/h] : 0.2777[m\s]
		va.pose.orientation.x = vn(0);
		va.pose.orientation.y = vn(1);
		va.pose.orientation.z = vn(2);
		va.pose.orientation.w = 0.0;
	}
	va.header.stamp = ros::Time::now();
}
template void VelocityArrow::setPoint<geometry_msgs::Point>(const geometry_msgs::Point&);

template<class T_p>
void VelocityArrow::setPoint2d(const T_p& p)
{
	v.block<2, 1>(0, 0) = diff.set(p.x, p.y);
	// va.scale.x = diff.norm();
	setNorm();
	va.pose.position.x = p.x;
	va.pose.position.y = p.y;
	va.pose.position.z = 0.0;
	geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromYaw(atan2(v(1), v(0)));
	va.pose.orientation = quat;
	va.header.stamp = ros::Time::now();
}
template void VelocityArrow::setPoint2d<geometry_msgs::Point>(const geometry_msgs::Point&);

void VelocityArrow::filter(const int& num)
{
	save = num;
}

visualization_msgs::Marker VelocityArrow::get() const
{
	return va;
}

void VelocityArrow::publish()
{
	if(!flag_pub){
		pub = n.advertise<visualization_msgs::Marker>(topic_name, 1);
		flag_pub = true;
	}
	va.header.stamp = ros::Time::now();
	pub.publish(va);
}

ostream& operator << (ostream &os, const VelocityArrow &va)
{
	os << va.va;

	return os;
}

//////////////// pritvate /////////////

void VelocityArrow::initialize()
{
	va.header.frame_id = "/world";
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
	va.header.stamp = ros::Time::now();
}

void VelocityArrow::setNorm()
{
	if(save > 1){
		if(filled){
			velocity_sum -= velocities[cnt];
			velocities[cnt] = diff.norm();
			velocity_sum += velocities[cnt];
			va.scale.x = velocity_sum / save;
		}else{
			velocities.push_back(diff.norm());
			velocity_sum += velocities[cnt];
			va.scale.x = velocity_sum / (cnt + 1);
			if(!(cnt+1 < save)){
				filled = true;
			}
		}
	}else{
		va.scale.x = diff.norm();
	}
	cnt = (cnt+1) % save;
}


