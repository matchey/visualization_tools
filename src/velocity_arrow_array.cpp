
//
// src: velocity_arrow_array.cpp
//
// last update: '18.1.26
// author: matchey
//
// memo:
//   Rviz表示用 Point(x, y, z)から速度ベクトルを表示
//

#include "visualization_tools/velocity_arrow_array.h"

using namespace std;

VelocityArrowArray::VelocityArrowArray()
	: topic_name("/velocity_arrows"), isBegin(true)
{
	pub = n.advertise<visualization_msgs::MarkerArray>(topic_name, 1);
}

void VelocityArrowArray::setTopicName(const string& str)
{
	topic_name = str;
	pub = n.advertise<visualization_msgs::MarkerArray>(topic_name, 1);
}

void VelocityArrowArray::publish()
{
	if(arrows.markers.size()){
		pub.publish(arrows);
	}
}

ostream& operator << (ostream &os, const VelocityArrowArray &va)
{
	os << va.arrows;

	return os;
}

//////////////// pritvate /////////////

