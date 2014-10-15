#ifndef ppr_Surface_H_
#define ppr_Surface_H_

#include <vector>

#include <stdio.h>

#include <Eigen/Core>
namespace ppr {
//Class representing some sort of surface such as plane or spheres
class Surface
{
	public:
	Surface();
	~Surface();
	virtual float distance(float x, float y, float z);															//Calculates the distance of a point to the surface
	virtual float angle(float x, float y, float z, float nx, float ny, float nz);								//Calculates the angle of a point+normal to the surface
	virtual void update(std::vector<float> & px, std::vector<float> & py, std::vector<float> & pz, std::vector<float> & pweight);	//Update surface params
	virtual void print();																						//Do I really nead to spell it out?
	virtual Surface * clone();																					//Create a new copy of the surface
	virtual void transform(Eigen::Matrix4f t);																	//Transform into other coordinate frame
};
}
#endif
