#ifndef ppr_RGBDSegmentModelFactory_H_
#define ppr_RGBDSegmentModelFactory_H_
#include "SegmentModelFactory.h"
#include <vector>

namespace ppr {

class RGBDSegmentModelFactory : public SegmentModelFactory
{
	public:
	std::vector<SegmentModelFactory * > factories;
	SegmentModelFactory * d_base;
	SegmentModelFactory * c_base;
	RGBDSegmentModelFactory();
	virtual void add(SegmentModelFactory * smf);
	virtual ~RGBDSegmentModelFactory();
	virtual std::vector<SegmentModel*> * getModels(	std::vector<int>  w, std::vector<int>  h, std::vector<float> d, float ** x, float ** y, float ** z, float ** r, float ** g, float ** b,float ** nx, float ** ny, float ** nz,SegmentModel * base);
	//virtual void setSurface(Surface * s);
	
	virtual void setDebugg(bool state);
	virtual void setVisualize(bool state);

	virtual float getExpectedInliers();
	virtual float getMaxDistance();
	virtual void setMaxDistance(float d);
	
	virtual void setFrameNr(int i);
	virtual void setIterationNr(int i);
};

}
#endif
