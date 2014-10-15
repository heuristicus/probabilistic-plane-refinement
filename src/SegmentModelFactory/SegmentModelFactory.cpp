#include "SegmentModelFactory.h"
//#include "mygeometry/mygeometry.h"
//#include <opencv.hpp>
namespace ppr {
SegmentModelFactory::SegmentModelFactory(){}
SegmentModelFactory::~SegmentModelFactory(){}

void SegmentModelFactory::setDebugg(bool state){	debugg = state;}
void SegmentModelFactory::setVisualize(bool state){	visualize = state;}
//void SegmentModelFactory::setSurface(Surface * s){	surface = s;}
vector<SegmentModel*> * SegmentModelFactory::getModels(	vector<int>  w, vector<int>  h, vector<float> d, float ** x, float ** y, float ** z, float ** r, float ** g, float ** b,float ** nx, float ** ny, float ** nz,SegmentModel * base){					return 0;}
float SegmentModelFactory::getExpectedInliers(){	return -1;}
float SegmentModelFactory::getMaxDistance(){		return -1;}
void SegmentModelFactory::setMaxDistance(float f){}
void SegmentModelFactory::setFrameNr(int i){		frame_nr = i;}
void SegmentModelFactory::setIterationNr(int i){	iteration_nr = i;}
}
