#include "ColorHistogramSegmentModel.h"
namespace ppr {
ColorHistogramSegmentModel::ColorHistogramSegmentModel(	int * color_bin_index_, float * hist_diff_){
	color_bin_index = color_bin_index_;
	hist_diff = hist_diff_;
}

ColorHistogramSegmentModel::~ColorHistogramSegmentModel(){}

float ColorHistogramSegmentModel::getModelFit(float d, float x, float y, float z, float r, float g, float b, float nx, float ny, float nz){					return hist_diff[color_bin_index[255*255*int(r)+255*int(g)+int(b)]];}
}
