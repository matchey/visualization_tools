
#ifndef BOUNDING_BOX_HPP
#define BOUNDING_BOX_HPP

#include <pcl_conversions/pcl_conversions.h>
#include <Eigen/Dense>
#include <tf/transform_broadcaster.h>

namespace visualization_tools
{
	template<class PointCloudPtr> void BoundingBox::
		vcovCalculator(const PointCloudPtr& pc, Eigen::Matrix2d &vcov)
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

	template<class PointCloudPtr>
	void BoundingBox::calcRange
	(const PointCloudPtr& pc, Eigen::Vector2d& vec, double &range, double &center, int axis)
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

	template<class PointCloudPtr>
	void BoundingBox::calcRange(const PointCloudPtr& pc, double &range)
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

	template<class PointCloudPtr>
	void BoundingBox::pca(const PointCloudPtr& pc)
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

	template<class PointCloudPtr>
	void BoundingBox::fromPCL(const PointCloudPtr& pc)
	{
		pca(pc);
		if(isBox) setBox();
		pcl_conversions::fromPCL(pc->header, bb.header);
	}
}

#endif

