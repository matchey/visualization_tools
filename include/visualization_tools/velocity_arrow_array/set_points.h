
#ifndef VA_ARRAY_SET_POINTS_H
#define VA_ARRAY_SET_POINTS_H

#include "visualization_tools/velocity_arrow_array.h"

typedef geometry_msgs::Pose gpose;

template<class... T_p>
void VelocityArrowArray::setPoints(T_p... args)
{
	int i = 0;
	arrows.markers.clear();
	for(gpose p : std::initializer_list<gpose>{args...}){
		if(isBegin){
			VelocityArrow va;
			va.setPoint(p);
			vas.push_back(va);
		}else{
			vas[i].setPoint(p);
			arrows.markers.push_back(vas[i++].get());
		}
	}
	if(isBegin) isBegin = false;
}

#endif

