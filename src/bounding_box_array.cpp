
//
// src: bounding_box_array.cpp
//
// last update: '18.1.10
// author: matchey
//
// memo:
//   Rviz表示用 BoundingBox作成Class
//

// #include <pcl/kdtree/impl/io.hpp>// for copyPointCloud
// #include <pcl/common/impl/io.hpp>// for copyPointCloud
#include "visualization_tools/bounding_box_array.h"

using std::ostream;
using std::cout;
using std::endl;

namespace visualization_tools
{
	// BoundingBoxArray::BoundingBoxArray(ros::NodeHandle node)
	BoundingBoxArray::BoundingBoxArray()
		:topic_name("/bounding_boxes"), frame_id("/velodyne"), box_size(0)
	{
		// bb = new BoundingBox();
		// n = node;

		pub = n.advertise<visualization_msgs::Marker>(topic_name, 10);

		bbs.header.frame_id = frame_id;
		// bbs.header.stamp = ros::Time::now();
		bbs.ns = "bounding_boxes";
		bbs.action = visualization_msgs::Marker::ADD;
		bbs.type = visualization_msgs::Marker::LINE_LIST;
		bbs.pose.orientation.w = 1.0;
		bbs.scale.x = 0.05;

		bbs.id = 0;

		bbs.color.r = 0.0;
		bbs.color.g = 1.0;
		bbs.color.b = 1.0;
		bbs.color.a = 0.7;

		bbs.lifetime = ros::Duration(0.1);
	}

	void BoundingBoxArray::setTopicName(const std::string& str)
	{
		topic_name = str;
		pub = n.advertise<visualization_msgs::Marker>(topic_name, 1);
	}

	void BoundingBoxArray::setFrameId(const std::string& str)
	{
		frame_id = str;
		bbs.header.frame_id = frame_id;
	}

	void BoundingBoxArray::push_back(const BoundingBox& b)
	{
		if(b.empty()) return;

		++box_size;
		visualization_msgs::Marker marker = b.getMarker();

		for(auto it = marker.points.begin(); it != marker.points.end(); ++it){
			bbs.points.push_back(*it);
		}

		v_bb.push_back(b);

		bbs.header = b.getMarker().header;
	}


	// void BoundingBoxArray::push_back(const pcl::PointCloud<pcl::PointXYZ>::Ptr& pc)
	// {
	// 	BoundingBox bb;
    //
	// 	bb.fromPCL(pc);
	// 	push_back(bb);
	// }
	// template<class PointT>
	// void BoundingBoxArray::push_back(const typename pcl::PointCloud<PointT>::Ptr& pc_in)
	// {
	// 	pcl::PointCloud<pcl::PointXYZ>::Ptr pc(new pcl::PointCloud<pcl::PointXYZ>);
    //
	// 	pcl::copyPointCloud(*pc_in, *pc);
    //
	// 	BoundingBox bb;
    //
	// 	bb.fromPCL(pc);
	// 	push_back(bb);
	// }
	// template void BoundingBoxArray::push_back
	// 	<pcl::PointXYZI>(const pcl::PointCloud<pcl::PointXYZI>::Ptr&);
	// template void BoundingBoxArray::push_back
	// 	<pcl::PointXYZRGB>(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr&);
	// template void BoundingBoxArray::push_back
	// 	<pcl::PointNormal>(const pcl::PointCloud<pcl::PointNormal>::Ptr&);
	// template void BoundingBoxArray::push_back
	// 	<pcl::PointXYZINormal>(const pcl::PointCloud<pcl::PointXYZINormal>::Ptr&);

	size_t BoundingBoxArray::size() const
	{
		return box_size;
	}

	visualization_msgs::Marker BoundingBoxArray::getMarker() const
	{
		return bbs;
	}

	void BoundingBoxArray::clear()
	{
		box_size = 0;
		bbs.points.clear();
		v_bb.clear();
	}

	void BoundingBoxArray::publish() const
	{
		if(box_size){
			pub.publish(bbs);
		}
	}

	ostream& operator << (ostream &os, const BoundingBoxArray& boxes)
	{
		os << "topic_name : " << boxes.topic_name << "\n" 
		   << "header.frame_id : " << boxes.frame_id << "\n"
		   << "size : " << boxes.box_size << "\n";

		for(auto it = boxes.v_bb.begin(); it != boxes.v_bb.end(); ++it){
			os << "  box[" << it - boxes.v_bb.begin() << "]\n" 
			   << *it << "\n";
		}
		os << endl;

		return os;
	}
}

