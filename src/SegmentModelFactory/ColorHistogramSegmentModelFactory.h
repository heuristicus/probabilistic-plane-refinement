#ifndef ppr_ColorHistogramSegmentModelFactory_H_
#define ppr_ColorHistogramSegmentModelFactory_H_
#include "SegmentModelFactory.h"
#include <vector>
namespace ppr {

class ColorHistogramSegmentModelFactory : public SegmentModelFactory
{
	public:
	int color_res;
	int		* color_bin_index;
	float	* color_hist_weight;
	float	* color_hist_count;
	ColorHistogramSegmentModelFactory();
	virtual ~ColorHistogramSegmentModelFactory();
	virtual std::vector<SegmentModel*> * getModels(	std::vector<int>  w, std::vector<int>  h, std::vector<float> d, float ** x, float ** y, float ** z, float ** r, float ** g, float ** b,float ** nx, float ** ny, float ** nz,SegmentModel * base);
};
}

#endif
