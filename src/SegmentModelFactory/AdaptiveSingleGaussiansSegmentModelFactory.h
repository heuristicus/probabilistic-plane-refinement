#ifndef ppr_AdaptiveSingleGaussiansSegmentModelFactory_H_
#define ppr_AdaptiveSingleGaussiansSegmentModelFactory_H_
#include "SegmentModelFactory.h"
#include <vector>

//using namespace std;
//using namespace pcl::visualization;
//using namespace Eigen;
namespace ppr {
class AdaptiveSingleGaussiansSegmentModelFactory : public SegmentModelFactory
{
	public:
	int smoothing;
	int bins;

	float max_d;
	int max_bins;
	int min_bins;
	int data_per_bin;

	double smoothng_std;
	double x_arr[10000];
	double smoothing_kernel[10000];

	float expectedInliers;
	
	float P_enforcement;

	AdaptiveSingleGaussiansSegmentModelFactory();
	AdaptiveSingleGaussiansSegmentModelFactory(float md);
	virtual ~AdaptiveSingleGaussiansSegmentModelFactory();
	virtual vector<SegmentModel*> * getModels(	std::vector<int>  w, std::vector<int>  h, std::vector<float> d, float ** x, float ** y, float ** z, float ** r, float ** g, float ** b,float ** nx, float ** ny, float ** nz,SegmentModel * base);

	void generate_d_distr(double * d_distr, std::vector<int> & w, std::vector<int> & h, std::vector<float> & d, float ** z, float max_d );
	void smooth_signal(double * d_distr_smooth, double * d_distr, double * smoothing_kernel);
	std::vector<int> find_maximas_and_minimas(double * d_distr_smooth);

	std::pair <double,double> fitGaussian(std::vector<float> value, std::vector<float> index);
	std::pair <double,double> fitGaussian(std::vector<float> value, std::vector<float> index, float mu);
	double gausian_error(std::vector<float> value, std::vector<float> index, double sigma, double mu);
	
	float getExpectedInliers();
	float getMaxDistance();
	void setMaxDistance(float f);
};
}
#endif
