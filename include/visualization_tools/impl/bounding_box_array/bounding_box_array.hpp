
#ifndef BOUNDING_BOX_ARRAY_HPP
#define BOUNDING_BOX_ARRAY_HPP

namespace visualization_tools
{
	template<typename PointCloudPtr>
	void BoundingBoxArray::push_back(const PointCloudPtr& pc)
	// void BoundingBoxArray::push_back(const typename pcl::PointCloud<PointCloudPtr>::Ptr& pc)
	{
		BoundingBox bb;
		// BoundingBox bb(n);

		bb.fromPCL(pc);
		push_back(bb);
		pcl_conversions::fromPCL(pc->header, bbs.header);
	}
}

#endif

