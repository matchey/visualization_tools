
//
// include: velocity_arrow.h
//
// last update: '18.1.26
// author: matchey
//
// memo:
//   Rviz表示用 Point(x, y, z)から速度ベクトルを表示
//

#ifndef VELOCITY_ARROW_H
#define VELOCITY_ARROW_H

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include "mmath/differential.h"

class VelocityArrow
{
	ros::NodeHandle n;
	ros::Publisher pub;

	visualization_msgs::Marker va; //Velocity Arrow
	std::string topic_name;
	Differential diff;
	Eigen::Vector3d v;

	static int id;

	public:
	VelocityArrow();
	void setTopicName(const std::string&);
	template<class T_p>
	void setPoint(const T_p&);
	template<class T_p>
	void setPoint2d(const T_p&);
	visualization_msgs::Marker get() const;
	void publish();

	friend std::ostream& operator << (std::ostream&, const VelocityArrow&);
};

#endif

