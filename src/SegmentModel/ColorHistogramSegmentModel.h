#ifndef ppr_ColorHistogramSegmentModel_H_
#define ppr_ColorHistogramSegmentModel_H_
#include "SegmentModel.h"
namespace ppr {
class ColorHistogramSegmentModel : public SegmentModel
{
	public:
	//Memmory owned by factory and borrowed by models
	int * color_bin_index;
	float * hist_diff;
	ColorHistogramSegmentModel( int * color_bin_index_, float * hist_diff_);
	~ColorHistogramSegmentModel();

	float getModelFit(float d, float x, float y, float z, float r, float g, float b, float nx, float ny, float nz);
};
}
#endif
