#ifndef ppr_SurfaceInference_H_
#define ppr_SurfaceInference_H_

namespace ppr {
//Selects inliers from likelihoods given by the ppr algorithm
class SurfaceInference
{
	public:
	int width;
	int height;
	float threshold;
	SurfaceInference();
	SurfaceInference(float t);
	virtual ~SurfaceInference();
	virtual void init(int width_, int height_, float ** x, float ** y, float ** z, float ** r, float ** g, float ** b, float ** nx, float ** ny, float ** nz); //Init per run of ppr algorithm
	virtual void infer(float ** prob, bool ** mask); //Infer inliers in the mask variable from the likelihoods in prob
};
}

#endif
