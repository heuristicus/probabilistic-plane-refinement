#include "MultiTypeSegmentModel.h"

namespace ppr {
MultiTypeSegmentModel::MultiTypeSegmentModel(){}
MultiTypeSegmentModel::~MultiTypeSegmentModel(){}

float MultiTypeSegmentModel::getModelFit(float d, float x, float y, float z, float r, float g, float b, float nx, float ny, float nz){
	float prod = 1;
	for(unsigned int i = 0; i < v.size(); i++){
		prod *= v.at(i)->getModelFit(d,x,y,z,r,g,b,nx,ny,nz);
	}
	return prod;
}

vector<float> MultiTypeSegmentModel::getAllFits(float d, float x, float y, float z, float r, float g, float b, float nx, float ny, float nz){
	vector<float> ret;
	for(unsigned int i = 0; i < v.size(); i++){
		ret.push_back(v.at(i)->getModelFit(d,x,y,z,r,g,b,nx,ny,nz));
	}
	return ret;
}
}
