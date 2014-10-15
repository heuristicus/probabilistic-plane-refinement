#include "RGBDSegmentModelFactory.h"
//#include "mygeometry/mygeometry.h"
namespace ppr {
RGBDSegmentModelFactory::RGBDSegmentModelFactory(){}
RGBDSegmentModelFactory::~RGBDSegmentModelFactory(){}

void RGBDSegmentModelFactory::add(SegmentModelFactory * smf){factories.push_back(smf);}
/*
void RGBDSegmentModelFactory::setSurface(Surface * s){
	surface = s;
	for(unsigned int j = 0; j < factories.size(); j++){factories.at(j)->setSurface(s);}
}
*/
void RGBDSegmentModelFactory::setDebugg(bool state)					{
	for(unsigned int j = 0; j < factories.size(); j++){factories.at(j)->setDebugg(state);}
	debugg = state;
}

void RGBDSegmentModelFactory::setVisualize(bool state)				{
	for(unsigned int j = 0; j < factories.size(); j++){factories.at(j)->setVisualize(state);}
	visualize = state;
}

vector<SegmentModel*> * RGBDSegmentModelFactory::getModels(	vector<int>  w, vector<int>  h, vector<float> d, float ** x, float ** y, float ** z, float ** r, float ** g, float ** b,float ** nx, float ** ny, float ** nz,SegmentModel * base){
	vector<SegmentModel*> * ret = new vector<SegmentModel*>();
	vector<SegmentModel*> * prev = 0;
	vector<SegmentModel*> * next = 0;
	for(unsigned int j = 0; j < factories.size(); j++){
		if(prev == 0){
			next = factories.at(j)->getModels(w,h,d,x,y,z,r,g,b,nx,ny,nz,0);
		}else{
			next = new vector<SegmentModel*>();
			for(unsigned int i = 0; i < prev->size(); i++){
				MultiTypeSegmentModel * multi = new MultiTypeSegmentModel();
				multi->v.push_back(prev->at(i));
				vector<SegmentModel*> * tmp = factories.at(j)->getModels(w,h,d,x,y,z,r,g,b,nx,ny,nz,prev->at(i));
				for(unsigned int k = 0; k < tmp->size(); k++){
					multi->v.push_back(tmp->back());
					tmp->pop_back();
				}
				delete tmp;
				next->push_back(multi);
			}
			delete prev;
		}
		prev = next;
	}
	return next;
}

void RGBDSegmentModelFactory::setFrameNr(int i){
	frame_nr = i;
	for(unsigned int j = 0; j < factories.size(); j++){factories.at(j)->setFrameNr(i);}
}
void RGBDSegmentModelFactory::setIterationNr(int i){
	iteration_nr = i;
	for(unsigned int j = 0; j < factories.size(); j++){factories.at(j)->setIterationNr(i);}
}

float RGBDSegmentModelFactory::getExpectedInliers(){
	float max_found = -1;
	for(unsigned int j = 0; j < factories.size(); j++){max_found = max(max_found,factories.at(j)->getExpectedInliers());}
	return max_found;
}

float RGBDSegmentModelFactory::getMaxDistance(){
	float max_found = -1;
	for(unsigned int j = 0; j < factories.size(); j++){max_found = max(max_found,factories.at(j)->getMaxDistance());}
	return max_found;
}

void RGBDSegmentModelFactory::setMaxDistance(float f){
	for(unsigned int j = 0; j < factories.size(); j++){factories.at(j)->setMaxDistance(f);}
}
}
