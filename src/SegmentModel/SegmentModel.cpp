#include "SegmentModel.h"


namespace ppr {

SegmentModel::SegmentModel(){}
SegmentModel::~SegmentModel(){}

float SegmentModel::getModelFit(float d, float x, float y, float z, float r, float g, float b, float nx, float ny, float nz){return 0;}
vector<float> SegmentModel::getAllFits(float d, float x, float y, float z, float r, float g, float b, float nx, float ny, float nz){return vector<float>();}
void SegmentModel::print(){printf("segment model\n");}

}
