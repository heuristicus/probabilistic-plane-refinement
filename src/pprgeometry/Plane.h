#ifndef ppr_Plane_H_
#define ppr_Plane_H_

#include <pcl/io/io.h>

#include <Eigen/Core>
#include <iostream>
#include <vector>
#include <Eigen/LU>
#include <Eigen/Eigenvalues> 
#include <Eigen/Eigen>
#include <Eigen/SVD>
#include <Eigen/Core>
#include "Surface.h"
namespace ppr {
//Plane class, see surface superclass for comments
class Plane : public Surface
{
	public:
	int id;

	//Simple plane representation
	double normal_x;
	double normal_y;
	double normal_z;
	double point_x;
	double point_y;
	double point_z;

	Plane(std::vector<float> & px, std::vector<float> & py, std::vector<float> & pz, std::vector<float> & pweight);
	Plane();
	
	~Plane();

	void update(std::vector<float> & px, std::vector<float> & py, std::vector<float> & pz, std::vector<float> & pweight);
	
	float distance(float x, float y, float z);
	double angle(Plane * p);//Angle between two planes
	float angle(float nx, float ny, float nz);
	float angle(float x, float y, float z, float nx, float ny, float nz);
	
	void print();
	Surface * clone();
	void transform(Eigen::Matrix4f t);
	void normalize();	//Normalize normal
	double norm();		//Get length of normal
};
}
#endif
