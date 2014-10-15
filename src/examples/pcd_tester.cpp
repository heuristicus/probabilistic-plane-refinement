
// PCL specific includes
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <pcl/io/io.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/cloud_viewer.h>


//Opencv
#include "cv.h" 
#include "highgui.h"
#include <opencv.hpp>

#include <string.h>


#include "../Refinement/SurfaceRefinement.h"



using namespace std;
using namespace pcl;
using namespace Eigen;

int counter = 0;
boost::shared_ptr<pcl::visualization::PCLVisualizer> myviewer;

bool next = false;
void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event, void* viewer_void){
	if (event.keyDown () && event.getKeySym () == "Right"){next = true;}
}


ppr::SurfaceRefinement * refinement;

ppr::Surface * segment(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud){
	int width = cloud->width;
	int height = cloud->height;
	pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
	normals->width = width;
	normals->height = height;
	normals->points.resize(width*height);

	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_normals(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	pcl::concatenateFields (*cloud, *normals, *cloud_normals);
	
	
	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr ransac_inliers (new pcl::PointIndices);
	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZRGBNormal> seg;
	seg.setOptimizeCoefficients (true);
	// Mandatory
	seg.setModelType (pcl::SACMODEL_PLANE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setDistanceThreshold (0.005);
	seg.setMaxIterations (1000);

	seg.setInputCloud (cloud_normals);
	seg.segment (*ransac_inliers, *coefficients);

	std::cerr << "Model inliers: " << ransac_inliers->indices.size () << std::endl;

	ppr::Plane * p = new ppr::Plane();
	p->normal_x = coefficients->values[0];
	p->normal_y = coefficients->values[1];
	p->normal_z = coefficients->values[2];
	float d = -coefficients->values[3];
	p->point_x	= p->normal_x*d;
	p->point_y	= p->normal_y*d;
	p->point_z	= p->normal_z*d;
	p->print();


	bool ** inliers = new bool*[width];
	for(int i = 0; i < width; i++){
		inliers[i] = new bool[height];
		for(int j = 0; j < height; j++){
			inliers[i][j] = true;
		}
	}

	for(int w = 0; w < width; w++){
		for(int h = 0; h < height; h++){
			PointXYZRGBNormal & point = cloud_normals->points.at(h*cloud->width+w);
			if(!isnan(point.z)){
				float d =  fabs(p->distance(point.x,point.y,point.z));
				inliers[w][h] = d < 0.005;
			}else{
				inliers[w][h] = false;
			}
		}
	}


	IplImage * inliers_img = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 3);
	for(int w = 0; w < width; w++){
		for(int h = 0; h < height; h++){
			if(inliers[w][h]){	cvRectangle(inliers_img,cvPoint(w, h),cvPoint(w, h),cvScalar(255.0,0,255.0, 0), 1 , 8, 0);}
			else{
				PointXYZRGBNormal & p = cloud_normals->points.at(h*width+w);			
				cvRectangle(inliers_img,cvPoint(w, h),cvPoint(w, h),cvScalar(p.b,p.g,p.r, 0), 1 , 8, 0);
			}
		}
	}

	cvShowImage("inliers_img", inliers_img);
	cvWaitKey(0);
	cvReleaseImage( &inliers_img );

	
	refinement->improve(inliers,cloud_normals,p);
	
	for(int w = 0; w < width; w++){
		for(int h = 0; h < height; h++){
			if(inliers[w][h]){
				int ind = h*width+w;
				cloud->points.at(ind).r = 255;
				cloud->points.at(ind).g = 0;
				cloud->points.at(ind).b = 255;
			}
		}
	}
	printf("START VIEWER STUFF\n");
	char buf[1024];
	sprintf(buf,"cloud");
	if(counter > 0){myviewer->removePointCloud(buf);}
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
	myviewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, buf);
	myviewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2.0, buf);
	while (!myviewer->wasStopped() && !next){
		myviewer->spinOnce (10);
		boost::this_thread::sleep (boost::posix_time::microseconds (100000));
	}
	next = false;

	counter++;		
	return p;
}

int main(int argc, char **argv){
	printf("starting pcd_tester software\n");
	
	refinement = new ppr::SurfaceRefinement();
	refinement->use_colors = true;
	refinement->setDebugg(false);
	refinement->setVisualize(false);
	refinement->setMaxDistance(0.04f);
	
 	myviewer = boost::shared_ptr<pcl::visualization::PCLVisualizer>(new pcl::visualization::PCLVisualizer ("Map viewer"));
	myviewer->setBackgroundColor (0,1,0);
	myviewer->initCameraParameters ();
	myviewer->registerKeyboardCallback (keyboardEventOccurred, (void*)&myviewer);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
	for(int i = 1; i < argc; i+=1){
		printf("path: %s\n",argv[i]);
		if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (argv[i], *cloud) != -1){
			segment(cloud);
		}
	}

	return 0;
}
