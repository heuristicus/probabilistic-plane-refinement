#include "SurfaceInference.h"
#include "pprgeometry/pprgeometry.h"
namespace ppr {
SurfaceInference::SurfaceInference(){threshold = 0.5;}
SurfaceInference::SurfaceInference(float t){threshold = t;}
SurfaceInference::~SurfaceInference(){}

void SurfaceInference::init(int width_, int height_,float ** x, float ** y, float ** z, float ** r, float ** g, float ** b, float ** nx, float ** ny, float ** nz){
	width = width_;
	height = height_;
}

void SurfaceInference::infer(float ** prob, bool ** mask){
	for(int w = 0; w < width; w++){
		for(int h = 0; h < height; h++){
			if(prob[w][h] >= threshold){	mask[w][h] = true;}
			else{							mask[w][h] = false;}
		}
	}
}
}
