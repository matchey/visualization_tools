
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
	// std::string frame_id;
	Differential diff;
	Eigen::Vector3d v;
	int save;
	int cnt;
	bool filled;
	double velocity_sum;
	std::vector<double> velocities;
	bool flag_pub;

	static int id;

	void initialize();
	void setNorm();

	public:
	VelocityArrow();
	void setHeader(const std_msgs::Header&);
	void setTime(const ros::Time&);
	void setFrameID(const std::string&);
	void setTopicName(const std::string&);
	void setPoint(const geometry_msgs::Pose&); //Pose
	template<class T_p>
	void setPoint(const T_p&); //Point
	template<class T_p>
	void setPoint2d(const T_p&); //Point
	void filter(const int&);
	visualization_msgs::Marker get() const;
	void publish();

	friend std::ostream& operator << (std::ostream&, const VelocityArrow&);
};

#endif

