
//
// include: bounding_box_array.h
//
// last update: '18.1.10
// author: matchey
//
// memo:
//   Rviz表示用 BoundingBox作成Class
//

#ifndef BOUNDING_BOX_ARRAY_H
#define BOUNDING_BOX_ARRAY_H

// #include <ros/ros.h>
#include "visualization_tools/bounding_box.h"

class BoundingBoxArray
{
	ros::NodeHandle n;
	ros::Publisher pub;

	visualization_msgs::Marker bbs;// Bounding Boxes

	std::string topic_name;
	std::string frame_id;
	int box_size;

	// BoundingBox* bb;
	std::vector<BoundingBox> v_bb;

	public:
	BoundingBoxArray();
	void setTopicName(std::string);
	void setFrameID(std::string);
	void push_back(BoundingBox);
	void push_back(pcl::PointCloud<pcl::PointXYZ>::Ptr&);
	template<class T_pc>
	void push_back(T_pc);
	void clear();
	void publish();

	friend std::ostream& operator << (std::ostream&, const BoundingBoxArray&);
};

#endif

