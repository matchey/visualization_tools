
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

class BoundingBox
{
	ros::NodeHandle n;
	ros::Publisher pub;

	visualization_msgs::Marker bb;// Bounding Box

	std::string topic_name;
	bool isBox;

	geometry_msgs::Pose bb_pose;
	double bb_height;
	double bb_width;
	double bb_depth;
	geometry_msgs::Point bb_center;
	double curv;

	void vcovCalculator(const pcl::PointCloud<pcl::PointXYZ>::Ptr&, Eigen::Matrix2d&);
	void calcRange(const pcl::PointCloud<pcl::PointXYZ>::Ptr&, Eigen::Vector2d&, double&, double&, int);
	void calcRange(const pcl::PointCloud<pcl::PointXYZ>::Ptr&, double&);
	void pca(const pcl::PointCloud<pcl::PointXYZ>::Ptr&);
	void transform();
	void setPose();
	void setBox();
	
	public:
	BoundingBox();
	bool empty();
	void setTopicName(std::string);
	void setFrameId(std::string);
	void setPose(geometry_msgs::Pose);
	void setSize(const double&, const double&, const double&); //height, width, depth
	void setCenter(const double&, const double&, const double&); //x, y, z
	visualization_msgs::Marker getMarker();
	void pc2bb(const pcl::PointCloud<pcl::PointXYZ>::Ptr&); //PointCloud to Bounding Box
	template<class T_p>
	void pc2bb(const T_p&);
	double height();
	double width();
	double depth();
	double curvature();
	void publish();

	friend std::ostream& operator << (std::ostream&, const BoundingBox&);
};

#endif

