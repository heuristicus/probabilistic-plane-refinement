#ifndef ppr_SegmentModelFactory_H_
#define ppr_SegmentModelFactory_H_
#include "SegmentModel.h"
#include <math.h>
#include <vector>

namespace ppr {
class SegmentModelFactory
{
	public:
	//Surface * surface;
	SegmentModelFactory();
	//virtual void setSurface(Surface * s);
	virtual ~SegmentModelFactory();
	virtual vector<SegmentModel*> * getModels(	std::vector<int>  w, std::vector<int>  h, std::vector<float> d, float ** x, float ** y, float ** z, float ** r, float ** g, float ** b,float ** nx, float ** ny, float ** nz,SegmentModel * base);
	
	int frame_nr;
	int iteration_nr;
	bool debugg;
	bool visualize;
		
	virtual void setDebugg(bool state);
	virtual void setVisualize(bool state);
	
	virtual float getExpectedInliers();
	virtual float getMaxDistance();
	virtual void setMaxDistance(float d);
	
	virtual void setFrameNr(int i);
	virtual void setIterationNr(int i);
};

}

#include "AdaptiveSingleGaussiansSegmentModelFactory.h"
#include "RGBDSegmentModelFactory.h"
#include "ColorHistogramSegmentModelFactory.h"

#endif
