
#ifndef VA_ARRAY_SET_POINTS_H
#define VA_ARRAY_SET_POINTS_H

#include "visualization_tools/velocity_arrow_array.h"

// typedef geometry_msgs::Pose gpose;
// typedef geometry_msgs::Point gpoint;

template<class First, class... T_p>
void VelocityArrowArray::push_back(const First& first, const T_p&... p)
{
	if(isBegin){
		VelocityArrow va;
		if(save > 1){
			va.filter(save);
		}
		va.setPoint(first);
		vas.push_back(va);
	}else{
		vas[count].setPoint(first);
		arrows.markers.push_back(vas[count++].get());
	}

	push_back(p...);
}

template<class... T_a>
void VelocityArrowArray::setPoints(const T_a&... args)
{
	arrows.markers.clear();

	// for(T_p p : std::initializer_list<T_p>{args...}){
	// for(gpose p : std::initializer_list<gpose>{args...}){
	// for(gpoint p : std::initializer_list<gpoint>{args...}){
	// 	if(isBegin){
	// 		VelocityArrow va;
	// 		va.setPoint(p);
	// 		vas.push_back(va);
	// 	}else{
	// 		vas[i].setPoint(p);
	// 		arrows.markers.push_back(vas[i++].get());
	// 	}
	// }
	
	count = 0;
	push_back(args...);

	if(isBegin) isBegin = false;
}

#endif

