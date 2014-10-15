#ifndef ppr_SurfaceRefinement_H_
#define ppr_SurfaceRefinement_H_

#include <opencv.hpp>
#include <stdlib.h>
#include <math.h>
#include <sys/types.h>
#include <sys/time.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>

#include <string>
#include <iostream>
#include <stdio.h>

#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>

#include <pcl/console/parse.h>

#include "../pprgeometry/pprgeometry.h"
#include "../SegmentModel/SegmentModel.h"
#include "../SegmentModelFactory/SegmentModelFactory.h"
#include "../SegmentModelFactory/AdaptiveSingleGaussiansSegmentModelFactory.h"
#include "../SurfaceInference/SurfaceInference.h"

namespace ppr {

using namespace Eigen;
using namespace std;
using namespace pcl;



class SurfaceRefinementDebugg{
	public:
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr inliers;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr probs;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr colprobs;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr geoprobs;
};

class SurfaceRefinement
{
	public:
		
		boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
		
		vector<SurfaceRefinementDebugg * > debuggdata;
		bool save;
		void saveDebugg(bool s);


		bool debugg;
		bool visualize;
		
		bool rescale;
		
		int width,height;
		
		float ** r;
		float ** g;
		float ** b;

		float ** x;
		float ** y;
		float ** z;

		float ** nx;
		float ** ny;
		float ** nz;
	
		float ** distance;
		float ** probability;
	
		bool ** mask;
		
		bool weighted_estimation;
		
		bool use_normals;
		bool use_colors;


		enum distance_type { EUCLIDEAN, SENSORNOISE} type;

		//Main loop
		int iterations;
		float max_distance;
		
		void startViewer();
		void trackershow();
		
		SegmentModelFactory * modelFactory;
		SurfaceInference * inference;
		
		AdaptiveSingleGaussiansSegmentModelFactory * tmp1;
		
		SurfaceRefinement();
		virtual ~SurfaceRefinement();
		virtual PointCloud<PointXYZRGBNormal>::Ptr improve(bool ** inliers, PointCloud<PointXYZRGBNormal>::Ptr input_cloud, Surface * surface);
		virtual void setDebugg(bool state);
		virtual void setVisualize(bool state);
		virtual void setInference(SurfaceInference * si);
		virtual void setType(distance_type t);
		virtual void setMaxDistance(float f);
		
		//virtual void calcEdges(int width, int height, float ** edges_w, float ** edges_h,float ** z, float ** r, float ** g, float ** b);
};

}
#endif
