
//
// include: bounding_box.h
//
// last update: '18.1.10
// author: matchey
//
// memo:
//   Rviz表示用 BoundingBox作成Class
//

#ifndef BOUNDING_BOX_H
#define BOUNDING_BOX_H

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Pose.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Core>

namespace visualization_tools
{
	class BoundingBox
	{
		ros::NodeHandle n;
		ros::Publisher pub;

		visualization_msgs::Marker bb;// Bounding Box

		std::string topic_name;
		bool isBox;

		geometry_msgs::Pose bb_pose;
		double bb_head;
		double bb_height;
		double bb_width;
		double bb_depth;
		geometry_msgs::Point bb_center;
		double curv;

		template<class PointCloudPtr>
		void vcovCalculator(const PointCloudPtr&, Eigen::Matrix2d&);
		// void vcovCalculator(const typename pcl::PointCloud<PointCloudPtr>::Ptr&, Eigen::Matrix2d&);
		template<class PointCloudPtr> void calcRange
			(const PointCloudPtr&, Eigen::Vector2d&, double&, double&, int);
		template<class PointCloudPtr>
		void calcRange(const PointCloudPtr&, double&);
		// void calcRange(const typename pcl::PointCloud<PointCloudPtr>::Ptr&, double&);
		template<class PointCloudPtr>
		void pca(const PointCloudPtr&);
		// void pca(const typename pcl::PointCloud<PointCloudPtr>::Ptr&);
		void transform();
		void setPose();
		void setBox();
		
		public:
		// BoundingBox(ros::NodeHandle);
		BoundingBox();
		geometry_msgs::Pose pose() const;
		bool empty() const;
		void setTime(const ros::Time&);
		void setTopicName(std::string);
		void setFrameId(std::string);
		void setPose(geometry_msgs::Pose);
		void setSize(const double&, const double&, const double&); //height, width, depth
		void setCenter(const double&, const double&, const double&); //x, y, z
		visualization_msgs::Marker getMarker() const;
		// void fromPCL(const pcl::PointCloud<pcl::PointXYZ>::Ptr&); //PointCloud to Bounding Box
		template<class PointCloudPtr>
		void fromPCL(const PointCloudPtr&);
		// void fromPCL(const typename pcl::PointCloud<PointCloudPtr>::Ptr&);
		double head() const;
		double height() const;
		double width() const;
		double depth() const;
		double curvature() const;
		void publish() const;

		friend std::ostream& operator << (std::ostream&, const BoundingBox&);
	};
}
#include "visualization_tools/impl/bounding_box/bounding_box.hpp"

#endif

