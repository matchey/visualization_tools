
//
// src: bounding_box.cpp
//
// last update: '18.1.10
// author: matchey
//
// memo:
//   Rviz表示用 BoundingBox作成Class
//

#include <tf/transform_broadcaster.h>
#include <pcl/kdtree/impl/io.hpp> 
#include <Eigen/Dense>
#include "visualization_tools/bounding_box.h"

using namespace std;

BoundingBox::BoundingBox()
	:topic_name("/bounding_box"), isBox(false), bb_height(1.7), bb_width(0.5), bb_depth(0.3)
{
	pub = n.advertise<visualization_msgs::Marker>(topic_name, 1);

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

bool BoundingBox::empty()
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

void BoundingBox::setTopicName(string str)
{
	topic_name = str;
	pub = n.advertise<visualization_msgs::Marker>(topic_name, 1);
}

void BoundingBox::setPose(geometry_msgs::Pose pose)
{
	bb_pose = pose;
	setBox();
}

void BoundingBox::vcovCalculator(const pcl::PointCloud<pcl::PointXYZ>::Ptr &pc, Eigen::Matrix2d &vcov)
{
	int n = pc->points.size();
	if(n < 4){ // 3点だと平面(Boxにならない)
		// cout << "!!!!! EMPTY CLOUD (in pca) !!!!!" << endl;
		isBox = false;
		return;
	}
	isBox = true;
	double s_xx, s_xy, s_yy;
	double x_sum, y_sum, x_ave, y_ave, xx_sum, yy_sum, xy_sum;

	x_sum = y_sum = x_ave = y_ave = xx_sum = yy_sum = xy_sum = 0.0;

	for(auto it = pc->points.begin(); it != pc->points.end(); ++it){
		x_sum += it->x;
		y_sum += it->y;
		xx_sum += it->x * it->x;
		yy_sum += it->y * it->y;
		xy_sum += it->x * it->y;
	}   
	x_ave = x_sum / n;
	y_ave = y_sum / n;

	s_xx = xx_sum / n - x_ave * x_ave;
	s_yy = yy_sum / n - y_ave * y_ave;
	s_xy = xy_sum / n - x_ave * y_ave;

	vcov << s_xx, s_xy,
		    s_xy, s_yy;

}

void BoundingBox::calcRange(const pcl::PointCloud<pcl::PointXYZ>::Ptr &pc, Eigen::Vector2d& vec, double &range, double &center, int axis)
{
	Eigen::Vector2d point;
	double min, max;
	double min_roatated, max_roatated;
	double theta = -atan2(vec(1), vec(0));
	Eigen::Rotation2D<double> rot(theta);

	point << pc->points[0].x, pc->points[0].y;
	min = max = point(axis);
	point = rot * point;
	min_roatated = max_roatated = point(0);

	for(auto it = pc->points.begin() + 1; it != pc->points.end(); ++it){
		point << it->x, it->y;
		if(point(axis) < min){
			min = point(axis);
		}
		if(max < point(axis)){
			max = point(axis);
		}
		point = rot * point;
		if(point(0) < min_roatated){
			min_roatated = point(0);
		}
		if(max_roatated < point(0)){
			max_roatated = point(0);
		}
	}

	range = max_roatated - min_roatated;
	center = (max + min) / 2.0;
}

void BoundingBox::calcRange(const pcl::PointCloud<pcl::PointXYZ>::Ptr &pc, double &range)
{
	double min, max;

	min = max = pc->points[0].z;

	for(auto it = pc->points.begin() + 1; it != pc->points.end(); ++it){
		if(it->z < min){
			min = it->z;
		}
		if(max < it->z){
			max = it->z;
		}
	}

	range = max - min;
	bb_center.z = min;
}

void BoundingBox::pca(const pcl::PointCloud<pcl::PointXYZ>::Ptr &pc)
{

	Eigen::Matrix2d vcov;

	vcovCalculator(pc, vcov);
	if(!isBox) return;

	Eigen::EigenSolver<Eigen::Matrix2d> es(vcov);
	Eigen::Vector2d values = es.eigenvalues().real();
	Eigen::Matrix2d vectors = es.eigenvectors().real();

	int min = values(0) < values(1) ? 0 : 1;
	int	max = 1 - min;

	double sum = values(0) + values(1);

	if(sum < 1e-6){
		curv = 1.0;
	}else{
		curv = values(min) / sum;
	}

	Eigen::Vector2d vec;

	vec = vectors.col(max);
	calcRange(pc, vec, bb_width, bb_center.y, 1);
	// calcRange(pc, vectors.col(max), bb_width, bb_center.y);
	vec = vectors.col(min);
	calcRange(pc, vec, bb_depth, bb_center.x, 0);
	// calcRange(pc, vectors.col(min), bb_depth, bb_center.x);
	calcRange(pc, bb_height);// 3回呼び出さないほうが早い -> 1回のfor文にまとめる

	bb_pose.position = bb_center;
	bb_pose.orientation = tf::createQuaternionMsgFromYaw(atan2(vec(1), vec(0)));
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
}

visualization_msgs::Marker BoundingBox::getMarker()
{
	return bb;
}

void BoundingBox::pc2bb(const pcl::PointCloud<pcl::PointXYZ>::Ptr &pc)
{
	pca(pc);
	if(isBox) setBox();
}

template<class T_p>
void BoundingBox::pc2bb(const T_p& p)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr pc(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::copyPointCloud(*p, *pc);
	pc2bb(pc);
}
template void BoundingBox::pc2bb<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr&);

double BoundingBox::height()
{
	return bb_height;
}

double BoundingBox::width()
{
	return bb_width;
}

double BoundingBox::depth()
{
	return bb_depth;
}

double BoundingBox::curvature()
{
	return curv;
}

void BoundingBox::publish()
{
	if(isBox){
		// setBox();
		bb.header.stamp = ros::Time::now();

		pub.publish(bb);
	}
}

ostream& operator << (ostream &os, const BoundingBox &box)
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

