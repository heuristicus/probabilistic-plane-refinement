#ifndef ppr_SegmentModel_H_
#define ppr_SegmentModel_H_

#include <vector>
#include <stdio.h>
namespace ppr {

using namespace std;


class SegmentModel
{
	public:

	SegmentModel();
	virtual ~SegmentModel();
	virtual float getModelFit(float d, float x, float y, float z, float r, float g, float b, float nx, float ny, float nz);//Checks how likeley a measurement is to fit a surface model
	virtual vector<float> getAllFits(float d, float x, float y, float z, float r, float g, float b, float nx, float ny, float nz);
	virtual void print();
};

}

#include "MultiTypeSegmentModel.h"
#include "GaussiansSegmentModel.h"
#include "ColorHistogramSegmentModel.h"

#endif
