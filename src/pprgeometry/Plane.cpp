#include "Plane.h"
#include <iostream>
#include <vector>
#include <algorithm>

#include <Eigen/LU>
#include <Eigen/Eigenvalues> 
#include <Eigen/Eigen>
#include <Eigen/SVD>
#include <Eigen/Core>
namespace ppr {
using namespace Eigen;
using namespace std;
int plane_counter = 0;

Plane::Plane(std::vector<float> & px, std::vector<float> & py, std::vector<float> & pz, std::vector<float> & pweight){
	id = plane_counter++;
	update(px,py,pz,pweight);
}

void Plane::update(vector<float> & px, vector<float> & py, vector<float> & pz, vector<float> & pweight){
	const unsigned int nr_points = px.size();

	float * x = new float[nr_points];
	float * y = new float[nr_points];
	float * z = new float[nr_points];
	float * weights = new float[nr_points];
	
	float tot_w = 0;
	point_x =0;
	point_y =0;
	point_z =0;

	for(unsigned int i = 0; i < nr_points; i++){
		float tmp_x = px.at(i);
		float tmp_y = py.at(i);
		float tmp_z = pz.at(i);
		float tmp_w = pweight.at(i);
	
		point_x+= tmp_w*tmp_x;
		point_y+= tmp_w*tmp_y;
		point_z+= tmp_w*tmp_z;
		
		x[i] = tmp_x;
		y[i] = tmp_y;
		z[i] = tmp_z;
		weights[i] = tmp_w;
		
		tot_w += tmp_w;
	}

	point_x/=tot_w;
	point_y/=tot_w;
	point_z/=tot_w;
	
	for(unsigned int i = 0; i < nr_points; i++){
		x[i] -= point_x;
		y[i] -= point_y;
		z[i] -= point_z;
	}

	vector<float *> data;
	data.push_back(x);
	data.push_back(y);
	data.push_back(z);
	
	unsigned int dim = data.size();
	
	MatrixXf covMat(dim,dim);

	for(unsigned int i = 0; i < dim; i++){
		for(int unsigned j = i; j < dim; j++){
			float * col1 	= data.at(i);
			float * col2 	= data.at(j);
			float sum = 0;
			for(unsigned int k = 0; k < nr_points; k++){
				sum += weights[k]*col1[k]*col2[k];
			}
			covMat(i,j)=sum/tot_w;
			covMat(j,i)=covMat(i,j);
		}
	}
	
	JacobiSVD<MatrixXf> svd(covMat, ComputeThinU | ComputeThinV);

	VectorXf S = svd.singularValues();

	float weight = (S(0)+S(1)+S(2))/S(2);

	if(isnan(weight)){weight = -1;}

	MatrixXf U = svd.matrixU();
	U = -U;
	
	normal_x 			= U(0,2);
	normal_y 			= U(1,2);
	normal_z 			= U(2,2);
	if( normal_z < 0){
		normal_x*=-1;
		normal_y*=-1;
		normal_z*=-1;
	}

	delete x;
	delete y;
	delete z;
	delete weights;

}

Plane::Plane(){id = plane_counter++;}

Plane::~Plane(){}

float Plane::distance(float x, float y, float z){	return (normal_x*(point_x-x) + normal_y*(point_y-y) + normal_z*(point_z-z));}

float Plane::angle(float x, float y, float z, float nx, float ny, float nz){	return angle(nx,ny,nz);}

float Plane::angle(float nx, float ny, float nz){
	if(isnan(nx)){return 1;}
	return normal_x*nx + normal_y*ny+normal_z*nz;
}

double Plane::norm(){return sqrt(normal_x*normal_x+normal_y*normal_y+normal_z*normal_z);}

void Plane::normalize(){
	double sum = norm();
	normal_x /= sum;
	normal_y /= sum;
	normal_z /= sum;
}

double Plane::angle(Plane * pl){
	pl->normalize();
	normalize();
	
	double dot = normal_x*pl->normal_x + normal_y*pl->normal_y+normal_z*pl->normal_z;
	return dot;
}

void Plane::print(){printf("Plane normals: %f %f %f\n",normal_x,normal_y,normal_z);}

//Copy plane, Still new id
Surface * Plane::clone(){
	Plane * p = new Plane();
	p->normal_x = normal_x;
	p->normal_y = normal_y;
	p->normal_z = normal_z;
	p->point_x  =  point_x;
	p->point_y  =  point_y;
	p->point_z  =  point_z;

	return p;
}

void Plane::transform(Eigen::Matrix4f t){
	Eigen::Vector4f n = t*Eigen::Vector4f(normal_x,normal_y,normal_z,0);//Only use rotational component
	n.normalize();
	normal_x = n(0);
	normal_y = n(1);
	normal_z = n(2);

	Eigen::Vector4f p = t*Eigen::Vector4f(point_x,point_y,point_z,1);
	point_x = p(0);
	point_y = p(1);
	point_z = p(2);
}
}
