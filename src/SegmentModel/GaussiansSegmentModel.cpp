#include "GaussiansSegmentModel.h"
#include <math.h>

namespace ppr {

GaussiansSegmentModel::GaussiansSegmentModel(float mul_, float mean_, float std_, float max_d_, int bins_, double * background_){
	mul = mul_;
	mean = mean_;
	std = std_;
	max_d = max_d_;
	bins = bins_;
	background = background_;
	minval = 0.005;
}
GaussiansSegmentModel::~GaussiansSegmentModel(){}

float GaussiansSegmentModel::getModelFit(float d, float x, float y, float z, float r, float g, float b, float nx, float ny, float nz){
	if(d > max_d){return 0;}
	float G = mul*exp(-0.5*(d-mean)*(d-mean)/(std*std));
	float H = max(mul*minval,background[int(bins*((d+max_d)/(2*max_d)))]);
	return min(G/H,1.0f);
}

}
