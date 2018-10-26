
//
// include: velocity_arrow_array.h
//
// last update: '18.1.26
// author: matchey
//
// memo:
//   Rviz表示用 Point(x, y, z)から速度ベクトルを表示
//

#ifndef VELOCITY_ARROW_ARRAY_H
#define VELOCITY_ARROW_ARRAY_H

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include "visualization_tools/velocity_arrow.h"

class VelocityArrowArray
{
	ros::NodeHandle n;
	ros::Publisher pub;

	std::string topic_name;
	std::vector<VelocityArrow> vas;
	visualization_msgs::MarkerArray arrows;
	bool isBegin;
	int count;
	int save;

	void push_back();
	template<class F, class... T_p>
	void push_back(const F&, const T_p&...);

	public:
	VelocityArrowArray();
	template<class... T_a>
	void setPoints(const T_a&...);
	void setTopicName(const std::string&);
	void filter(const int&);
	void publish();

	friend std::ostream& operator << (std::ostream&, const VelocityArrowArray&);
};

#include "visualization_tools/impl/velocity_arrow_array/set_points.hpp"

#endif

