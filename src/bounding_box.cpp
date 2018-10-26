
//
// src: bounding_box.cpp
//
// last update: '18.1.10
// author: matchey
//
// memo:
//   Rviz表示用 BoundingBox作成Class
//

// #include <pcl/kdtree/impl/io.hpp> 
#include "visualization_tools/bounding_box.h"

using namespace std;

namespace visualization_tools
{
	// BoundingBox::BoundingBox(ros::NodeHandle node)
	BoundingBox::BoundingBox()
		: topic_name("/bounding_box"), isBox(false), bb_height(1.7), bb_width(0.5), bb_depth(0.3)
	{
		// n = node;
		pub = n.advertise<visualization_msgs::Marker>(topic_name, 10);

		bb.header.frame_id = "/velodyne";
		// bb.header.stamp = ros::Time::now();
		bb.ns = "bounding_box";
		bb.action = visualization_msgs::Marker::ADD;
		bb.type = visualization_msgs::Marker::LINE_LIST;
		bb.pose.orientation.w = 1.0;
		bb.scale.x = 0.05;

		bb.id = 0;

		bb.color.r = 0.0;
		bb.color.g = 1.0;
		bb.color.b = 1.0;
		bb.color.a = 0.7;

		bb.lifetime = ros::Duration(0.1);
	}

	geometry_msgs::Pose BoundingBox::pose() const
	{
		return bb_pose;
	}

	bool BoundingBox::empty() const
	{
		// bool flag = true;
		//
		// if(bb.points.size() == 24){
		// 	flag = false;
		// }
		//
		// return flag;
		return !isBox;
	}

	void BoundingBox::setTime(const ros::Time& t)
	{
		bb.header.stamp = t;
	}

	void BoundingBox::setTopicName(string str)
	{
		topic_name = str;
		pub = n.advertise<visualization_msgs::Marker>(topic_name, 1);
	}

	void BoundingBox::setFrameId(string str)
	{
		bb.header.frame_id = str;
	}

	void BoundingBox::setPose(geometry_msgs::Pose pose)
	{
		bb_pose = pose;
		setBox();
	}

	void BoundingBox::setSize(const double& h, const double& w, const double& d)
	{
		bb_height = h;
		bb_width  = w;
		bb_depth  = d;
		isBox = true;
		setBox();
	}

	void BoundingBox::setCenter(const double& x, const double& y, const double& z)
	{
		bb_center.x = x;
		bb_center.y = y;
		bb_center.z = z;
		setBox();
	}

	void BoundingBox::transform()
	{
		// double roll, pitch, yaw;
		// tf::Quaternion q(bb_pose.orientation.x,
		// 				 bb_pose.orientation.y,
		// 				 bb_pose.orientation.z,
		// 				 bb_pose.orientation.w);
		// tf::Matrix3x3 m(q);
		// m.getRPY(roll, pitch, yaw);
		Eigen::Quaterniond quat(bb_pose.orientation.w,
								bb_pose.orientation.x,
								bb_pose.orientation.y,
								bb_pose.orientation.z);

		Eigen::Vector3d pos, pos_roatated;

		for(auto it = bb.points.begin(); it != bb.points.end(); ++it){
			pos << it->x - bb_depth/2.0, it->y - bb_width/2.0, it->z;
			pos_roatated = quat * pos;
			pos = pos_roatated;

			pos << pos_roatated(0) + bb_pose.position.x,
				   pos_roatated(1) + bb_pose.position.y,
				   pos_roatated(2) + bb_pose.position.z;

			it->x = pos(0);
			it->y = pos(1);
			it->z = pos(2);
		}
	}

	void BoundingBox::setBox()
	{
		geometry_msgs::Point p;
		int sign[4] = {1, 1, 0, 0};// +, -, -, +

		bb.points.clear();
		bb.points.reserve(24);

		p.x = p.y = p.z = 0.0;

		for(int i = 0; i < 2; i++){
			for(int j = 0; j < 4; j++){
				bb.points.push_back(p);
				p.x = sign[j] * bb_depth;
				p.y = sign[(j+3)%4] * bb_width;
				bb.points.push_back(p);
			}
			p.x = p.y = 0.0;
			p.z = bb_height;
		}

		p.x = p.y = p.z = 0.0;

		for(int i = 0; i < 4; i++){
			p.x = sign[i] * bb_depth;
			p.y = sign[(i+3)%4] * bb_width;
			p.z = 0.0;
			bb.points.push_back(p);
			p.z = bb_height;
			bb.points.push_back(p);
		}

		transform();
		bb_head = bb_pose.position.z + bb_height;
	}

	visualization_msgs::Marker BoundingBox::getMarker() const
	{
		return bb;
	}

	double BoundingBox::head() const
	{
		return bb_head;
	}

	double BoundingBox::height() const
	{
		return bb_height;
	}

	double BoundingBox::width() const
	{
		return bb_width;
	}

	double BoundingBox::depth() const
	{
		return bb_depth;
	}

	double BoundingBox::curvature() const
	{
		return curv;
	}

	void BoundingBox::publish() const
	{
		if(isBox){
			pub.publish(bb);
		}
	}

	ostream& operator << (ostream &os, const BoundingBox& box)
	{
		double roll, pitch, yaw;
		tf::Quaternion q(box.bb_pose.orientation.x,
						 box.bb_pose.orientation.y,
						 box.bb_pose.orientation.z,
						 box.bb_pose.orientation.w);
		tf::Matrix3x3 m(q);
		m.getRPY(roll, pitch, yaw);

		if(yaw < 0.0){
			yaw += M_PI;
		}

		os << "    width  : " << box.bb_width  << "\n" 
		   << "    depth  : " << box.bb_depth  << "\n"
		   << "    height : " << box.bb_height << "\n"
		   << "    curvature : " << box.curv << "\n"
		   << "    position  : (" << box.bb_pose.position.x << ", " 
							  << box.bb_pose.position.y << ", " 
							  << box.bb_pose.position.z << ")\n"
		   // << "direction : (" << cos(yaw) << ", " 
			// 				  << sin(yaw) << ")" << endl;
		   << "    direction : "  << yaw * 180 / M_PI << endl;

		return os;
	}
}

