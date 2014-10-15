#ifndef ppr_MultiTypeSegmentModel_H_
#define ppr_MultiTypeSegmentModel_H_
#include "SegmentModel.h"
namespace ppr {
class MultiTypeSegmentModel : public SegmentModel
{
	public:

	vector<SegmentModel * > v;//Vector with learned segment models
	MultiTypeSegmentModel();
	~MultiTypeSegmentModel();

	float getModelFit(float d, float x, float y, float z, float r, float g, float b, float nx, float ny, float nz);
	vector<float> getAllFits(float d, float x, float y, float z, float r, float g, float b, float nx, float ny, float nz);
};
}
#endif
