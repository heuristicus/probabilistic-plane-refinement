#ifndef ppr_GaussiansSegmentModel_H_
#define ppr_GaussiansSegmentModel_H_
#include "SegmentModel.h"
namespace ppr {
class GaussiansSegmentModel : public SegmentModel
{
	public:
	float mul;
	float mean;
	float std;
	float max_d;
	double minval;//Regularization when the histogram is very small.
	int bins;
	double * background;

	GaussiansSegmentModel(float mul_, float mean_, float std_, float max_d_,int bins, double * background_);
	~GaussiansSegmentModel();

	float getModelFit(float d, float x, float y, float z, float r, float g, float b, float nx, float ny, float nz);
};
}
#endif
