
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

namespace visualization_tools
{
	class BoundingBoxArray
	{
		ros::NodeHandle n;
		ros::Publisher pub;

		visualization_msgs::Marker bbs;// Bounding Boxes

		std::string topic_name;
		std::string frame_id;
		size_t box_size;

		// BoundingBox* bb;
		std::vector<BoundingBox> v_bb;

		public:
		BoundingBoxArray();
		// BoundingBoxArray(ros::NodeHandle);
		void setTopicName(const std::string&);
		void setFrameId(const std::string&);
		void push_back(const BoundingBox&);
		// void push_back(const pcl::PointCloud<pcl::PointXYZ>::Ptr&);
		template<typename PointCloudPtr>
		void push_back(const PointCloudPtr&);
		// void push_back(const typename pcl::PointCloud<PointCloudPtr>::Ptr&);
		size_t size() const;
		visualization_msgs::Marker getMarker() const;
		void clear();
		void publish() const;

		friend std::ostream& operator << (std::ostream&, const BoundingBoxArray&);
	};
}
#include "visualization_tools/impl/bounding_box_array/bounding_box_array.hpp"

#endif

