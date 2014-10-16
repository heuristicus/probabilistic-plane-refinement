#include "SurfaceRefinement.h"
#include "../util/util.h"
namespace ppr {

using namespace std;

int showtype = 0;
int showNr = 0;
vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > inlier_clouds;
vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > prob_clouds;
vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > colprob_clouds;
vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > geoprob_clouds;

bool SurfaceRefinementNext = false;
bool updated = false;

void SurfaceRefinement::trackershow(){
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
	if(showtype == 0){
		cloud = inlier_clouds.at(showNr-1);
		printf("Inliers\n");
	}
	
	if(showtype == 1){
		cloud = prob_clouds.at(showNr-1);
		printf("prob\n");
	}
	
	if(showtype == 2){
		cloud = colprob_clouds.at(showNr-1);
		printf("colprob\n");
	}
	
	if(showtype == 3){
		cloud = geoprob_clouds.at(showNr-1);
		printf("geoprob\n");
	}
	
	viewer->removePointCloud("cloud");
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
	viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "cloud");
	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2.0, "cloud");
	while (!viewer->wasStopped() && !SurfaceRefinementNext && !updated){
		viewer->spinOnce (10);
		boost::this_thread::sleep (boost::posix_time::microseconds (100000));
	}
	SurfaceRefinementNext = false;
	if(updated){
		updated = false;
		trackershow();
	}
}

void SurfaceRefinementkeyboardEventOccurred (const pcl::visualization::KeyboardEvent &event, void* viewer_void){
	if (event.keyDown () && event.getKeySym () == "Right"){
		SurfaceRefinementNext = true;
	}
	if (event.keyDown () && event.getKeySym () == "Up"){
		printf("%s\n",event.getKeySym().c_str());
		showtype = (showtype+1)%4;
		printf("showtype: %i\n",showtype);
		updated = true;
	}

	if (event.keyDown () && event.getKeySym () == "Down"){
		printf("%s\n",event.getKeySym().c_str());
		showtype --;
		if(showtype < 0){showtype = 3;}
		printf("showtype: %i\n",showtype);
		updated = true;
	}
}

void SurfaceRefinement::startViewer(){
	viewer = boost::shared_ptr<pcl::visualization::PCLVisualizer>(new pcl::visualization::PCLVisualizer ("SurfaceRefinement viewer"));
	viewer->setBackgroundColor (0.5, 0.8, 0.5);
	viewer->initCameraParameters ();
	viewer->registerKeyboardCallback (SurfaceRefinementkeyboardEventOccurred, (void*)&viewer);
}

SurfaceRefinement::SurfaceRefinement(){
	debugg = true;
	visualize = true;
	rescale = false;
	weighted_estimation = true;
	
	use_normals = false;
	use_colors = true;
		
	type = EUCLIDEAN;

	//Main loop
	iterations = 30;
	max_distance = 0.04;

	RGBDSegmentModelFactory * smf = new RGBDSegmentModelFactory();
	tmp1 = new AdaptiveSingleGaussiansSegmentModelFactory(max_distance);
	smf->add(tmp1);

	//if(use_normals){smf->add(new NormalSegmentModelFactory());}
	if(use_colors){	smf->add(new ColorHistogramSegmentModelFactory());} //Asumes color is corretly aligned to depth
	modelFactory = smf;
	smf->setVisualize(false);
	smf->setDebugg(false);
	inference = new SurfaceInference(0.5f);
	//inference = new SurfaceInference(0.1);
	//inference = new SurfaceInferenceSmartSmoothing();
	//inference = new SurfaceInferenceGraphCut();

	save = debugg = visualize = false;
}
SurfaceRefinement::~SurfaceRefinement(){}

void SurfaceRefinement::setDebugg(bool state)					{	debugg = state;}
void SurfaceRefinement::setVisualize(bool state)				{	visualize = state;}
void SurfaceRefinement::setInference(SurfaceInference * si)	{	delete inference; inference = si;}
void SurfaceRefinement::setType(distance_type t)				{	type = t;}
void SurfaceRefinement::setMaxDistance(float f)				{
	max_distance = f;
	delete modelFactory;
	RGBDSegmentModelFactory * smf = new RGBDSegmentModelFactory();
	tmp1 = new AdaptiveSingleGaussiansSegmentModelFactory(max_distance);
	smf->add(tmp1);
	//if(use_normals){smf->add(new NormalSegmentModelFactory());}
	if(use_colors){	smf->add(new ColorHistogramSegmentModelFactory());} //Asumes color is corretly aligned to depth
	modelFactory = smf;
	modelFactory->setVisualize(false);
	modelFactory->setDebugg(false);
	modelFactory->setFrameNr(0);
	modelFactory->setIterationNr(0);

}


void SurfaceRefinement::saveDebugg(bool s){save = s;}

int improve_count = 0;



PointCloud<PointXYZRGBNormal>::Ptr SurfaceRefinement::improve(bool ** inliers, PointCloud<PointXYZRGBNormal>::Ptr cloud, Surface * surface){
	double currentTime = getCurrentTime();
	double start = currentTime;
	modelFactory->setFrameNr(modelFactory->frame_nr+1);
	//Set data structures
	width = cloud->width;
	height = cloud->height;
	r = new float*[width];
	g = new float*[width];
	b = new float*[width];

	x = new float*[width];
	y = new float*[width];
	z = new float*[width];

	nx = new float*[width];
	ny = new float*[width];
	nz = new float*[width];
	
	distance = new float*[width];
	probability = new float*[width];
	
	bool ** mask = new bool*[width];
	
	for(int w = 0; w < width; w++){
		r[w] = new float[height];
		g[w] = new float[height];
		b[w] = new float[height];
		
		x[w] = new float[height];
		y[w] = new float[height];
		z[w] = new float[height];
		
		nx[w] = new float[height];
		ny[w] = new float[height];
		nz[w] = new float[height];

		distance[w] = new float[height];
		probability[w] = new float[height];

		mask[w] = new bool[height];
	}
	
	for(int w = 0; w < width; w++){
		for(int h = 0; h < height; h++){
			PointXYZRGBNormal & p = cloud->points.at(h*width+w);
			r[w][h] = p.r;
			g[w][h] = p.g;
			b[w][h] = p.b;
			
			if(!isnan(p.x)){ 
				x[w][h] = p.x;
				y[w][h] = p.y;
				z[w][h] = p.z;
			}else{
				x[w][h] = 0;
				y[w][h] = 0;
				z[w][h] = 0;
			}
			
			nx[w][h]	= p.normal_x;
			ny[w][h]	= p.normal_y;
			nz[w][h]	= p.normal_z;
			
			mask[w][h]	= true;
			
			distance[w][h]		= 0;
			probability[w][h]	= 0;
		}
	}
	
	if(debugg){
		printf("max_distance: %f\n",max_distance);
		printf("-----------------------------------------------------\n");
		printf("Init data time       = %f\n",getCurrentTime()-currentTime);
		currentTime = getCurrentTime();
	}
	//Init
	//modelFactory->setSurface(surface);
	inference->init(width,height,x,y,z,r,g,b,nx,ny,nz);
	if(debugg){
		printf("M+I init time        = %f\n",getCurrentTime()-currentTime);;
		currentTime = getCurrentTime();
	}
	//Main loop
	double total_distance_time	= 0;
	double total_train_time		= 0;
	double total_inference_time = 0;
	double total_update_time	= 0;
	double start_loop = getCurrentTime();
	
	showNr = 0;
	inlier_clouds.clear();
	
	for(int it = 0; it < iterations; it++){
		modelFactory->setIterationNr(it);
		max_distance = modelFactory->getMaxDistance();
		if(debugg){//modelFactory->frame_nr == 1){
			modelFactory->setVisualize(true);
			modelFactory->setDebugg(true);
		}else {
			modelFactory->setVisualize(false);
			modelFactory->setDebugg(false);
		}
	
		vector<int>  v_w;
		vector<int>  v_h;
		vector<float> v_d;
		

		//Extract distances for all points to the plane.
		double start_distance_time = getCurrentTime();
		for(int w = 0; w < width; w++){
			for(int h = 0; h < height; h++){
				probability[w][h] = 0;
				if(z[w][h]!=0){	
					float d = surface->distance(x[w][h],y[w][h],z[w][h]);
					distance[w][h] = d;
					if(fabs(d) < max_distance /*&& (it > iterations/2 || inliers[w][h])*/ ){
						v_w.push_back(w);
						v_h.push_back(h);
						v_d.push_back(d);
					}
				}else{				distance[w][h] = 999999;}
			}
		}
//printf("v_d.size() = %i\n",v_d.size());
		
		total_distance_time += getCurrentTime()-start_distance_time;
		
		//Train models and calculate probability for possible inliers
		double start_train_time = getCurrentTime();
		vector<SegmentModel*> * models = modelFactory->getModels(v_w, v_h, v_d, x, y, z, r, g, b,nx,ny,nz, 0); //Learn models
		if(models->size() == 0){
			
			printf("no models found \n");
			for(int w = 0; w < width; w++){
				delete[] distance[w];
				delete[] probability[w];
		
				delete[] mask[w];
	
				delete[] r[w];
				delete[] g[w];
				delete[] b[w];
		
				delete[] x[w];
				delete[] y[w];
				delete[] z[w];
		
				delete[] nx[w];
				delete[] ny[w];
				delete[] nz[w];
			}
			delete[] mask;
	
			delete[] distance;
			delete[] probability;
	
			delete[] r;
			delete[] g;
			delete[] b;
		
			delete[] x;
			delete[] y;
			delete[] z;
		
			delete[] nx;
			delete[] ny;
			modelFactory->setMaxDistance(modelFactory->getMaxDistance()+0.05);
			return cloud;
		}
		SegmentModel * model = models->at(0);

		vector<float> protest;
		for(unsigned int j = 0; j < v_d.size(); j++){
			int w = v_w.at(j);	int h = v_h.at(j);
			probability[w][h] = model->getModelFit(v_d.at(j),x[w][h], y[w][h], z[w][h],r[w][h], g[w][h], b[w][h],nx[w][h], ny[w][h], nz[w][h]);
			protest.push_back(probability[w][h]);
		}
		total_train_time += getCurrentTime()-start_train_time;
		
		//Infer inliers from probability function of inliers and inference algorithm
		double start_inference_time = getCurrentTime();
		int expectedInliers = modelFactory->getExpectedInliers();// from integral over foreground distribution
		sort(protest.begin(),protest.end());
		float inlier_threshold =  protest.at(max(int(protest.size()-expectedInliers-1),0));//Find the threshold for the number of expected inliers

		inference->threshold = inlier_threshold;
		inference->infer(probability,mask);
		total_inference_time += getCurrentTime()-start_inference_time;

		//re estimate parameters for surface using inliers
		double start_update_time = getCurrentTime();
		vector<float> px;
		vector<float> py;
		vector<float> pz;
		vector<float> pweight;

		for(unsigned int j = 0; j < v_d.size(); j++){
			int w = v_w.at(j);
			int h = v_h.at(j);
			if(weighted_estimation){
				if(probability[w][h] > 0){
					px.push_back(x[w][h]);
					py.push_back(y[w][h]);
					pz.push_back(z[w][h]);
					pweight.push_back(probability[w][h]);//1.0f);
				}
			}else if(mask[w][h]){
				px.push_back(x[w][h]);
				py.push_back(y[w][h]);
				pz.push_back(z[w][h]);
				pweight.push_back(1.0f);
			}
		}
		
		
		if(it >= iterations-1){//Update iniliers at last iteration
			for(int w = 0; w < width; w++){
				for(int h = 0; h < height; h++){
					inliers[w][h] = mask[w][h];
				}
			}
		}
		
	if(false && debugg && it == iterations-1){
		model->print();
		MultiTypeSegmentModel * multitypesegmentmodel = (MultiTypeSegmentModel *)model;
		
	
		float ** geoprob = new float*[width];
		float ** colprob = new float*[width];
		float ** jointprob = new float*[width];
		for(int w = 0; w < width; w++){
			geoprob[w] = new float[height];
			colprob[w] = new float[height];
			jointprob[w] = new float[height];
			for(int h = 0; h < height; h++){
				geoprob[w][h] = 0;
				colprob[w][h] = 0;
				jointprob[w][h] = 0;
				

				
				float d = surface->distance(x[w][h],y[w][h],z[w][h]);
				vector<float> vec = model->getAllFits(d,x[w][h], y[w][h], z[w][h],r[w][h], g[w][h], b[w][h],nx[w][h], ny[w][h], nz[w][h]);
				geoprob[w][h] = vec.at(0);
				colprob[w][h] = vec.at(1);
				jointprob[w][h] = geoprob[w][h]*colprob[w][h];
			}
		}
		
		IplImage * geoprob_img		= cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 3);
		IplImage * colprob_img		= cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 3);
		IplImage * jointprob_img	= cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 3);
		for(int w = 0; w < width; w++){
			for(int h = 0; h < height; h++){
				float geop = pow(geoprob[w][h],1.0);
				float colp = pow(colprob[w][h],1.0);
				float jointp = geop-jointprob[w][h];

				float rv = 1;//r[w][h]/255.0f;
				float gv = 1;//g[w][h]/255.0f;
				float bv = 1;//b[w][h]/255.0f;

				cvRectangle(geoprob_img,cvPoint(w, h),cvPoint(w, h),	cvScalar(255.0*geop*rv,		255.0*geop*gv,		255.0*geop*bv, 0), 1 , 8, 0);
				cvRectangle(colprob_img,cvPoint(w, h),cvPoint(w, h),	cvScalar(255.0*colp*rv,		255.0*colp*gv,		255.0*colp*bv, 0), 1 , 8, 0);
				cvRectangle(jointprob_img,cvPoint(w, h),cvPoint(w, h),	cvScalar(255.0*jointp*rv,	255.0*jointp*gv,	255.0*jointp*bv, 0), 1 , 8, 0);
			}
		}
		
		cvShowImage("geoprob_img", geoprob_img);
		cvShowImage("colprob_img", colprob_img);
		cvShowImage("jointprob_img", jointprob_img);
		
		char buf[1024];
		
		sprintf(buf,"geoprob_img%i.png",improve_count);
		cvSaveImage(buf,geoprob_img);
			
		sprintf(buf,"colprob_img%i.png",improve_count);
		cvSaveImage(buf,colprob_img);

		sprintf(buf,"jointprob_img%i.png",improve_count);
		cvSaveImage(buf,jointprob_img);

		//cvWaitKey(0);
		
		IplImage * prob_img = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 3);
		IplImage * mask_img = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 3);
		for(int w = 0; w < width; w++){
			for(int h = 0; h < height; h++){
				float p = probability[w][h];
				cvRectangle(prob_img,cvPoint(w, h),cvPoint(w, h),cvScalar(255.0*p,255.0*p,255.0*p, 0), 1 , 8, 0);
				if(mask[w][h]){	cvRectangle(mask_img,cvPoint(w, h),cvPoint(w, h),cvScalar(255.0,0,255.0, 0), 1 , 8, 0);}
				else{			cvRectangle(mask_img,cvPoint(w, h),cvPoint(w, h),cvScalar(b[w][h], g[w][h], r[w][h], 0), 1 , 8, 0);}
				if(z[w][h] == 0){cvRectangle(mask_img,cvPoint(w, h),cvPoint(w, h),cvScalar(0,255,0, 0), 1 , 8, 0);}
			}
		}
			
		cvShowImage("prob_img", prob_img);
		cvShowImage("mask_img", mask_img);
		cvWaitKey(100);
			


		sprintf(buf,"prob_img%i.png",improve_count);
		cvSaveImage(buf,prob_img);
			
		sprintf(buf,"mask_img%i.png",improve_count);
		cvSaveImage(buf,mask_img);
			
		cvReleaseImage( &prob_img );
		cvReleaseImage( &mask_img );
	}
		
		
	if(save){//debugg && it % 10 == 0){// && it == iterations-1){
		surface->print();
		printf("improve_count: %i iteration: %i allow all:%i\n",improve_count,it,it > iterations/2);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr inlier_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
		inlier_cloud->width = width;
		inlier_cloud->height = height;
		inlier_cloud->points.resize(width*height);
			
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr prob_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
		prob_cloud->width = width;
		prob_cloud->height = height;
		prob_cloud->points.resize(width*height);
			
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr colprob_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
		colprob_cloud->width = width;
		colprob_cloud->height = height;
		colprob_cloud->points.resize(width*height);
			
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr geoprob_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
		geoprob_cloud->width = width;
		geoprob_cloud->height = height;
		geoprob_cloud->points.resize(width*height);
			
		for(int w = 0; w < width; w++){
			for(int h = 0; h < height; h++){
				int ind = h*width+w;
					
				float d = surface->distance(x[w][h],y[w][h],z[w][h]);
				vector<float> vec = model->getAllFits(d,x[w][h], y[w][h], z[w][h],r[w][h], g[w][h], b[w][h],nx[w][h], ny[w][h], nz[w][h]);
				float geoprob = vec.at(0);
				//float colprob[w][h] = vec.at(1);
				
				float prob = probability[w][h];
				
				inlier_cloud->points[ind].x = x[w][h];
				inlier_cloud->points[ind].y = y[w][h];
				inlier_cloud->points[ind].z = z[w][h];
				inlier_cloud->points[ind].r	= r[w][h];
				inlier_cloud->points[ind].g = g[w][h];
				inlier_cloud->points[ind].b = b[w][h];
				if(mask[w][h]){
					inlier_cloud->points.at(ind).r = 255;
					inlier_cloud->points.at(ind).g = 0;
					inlier_cloud->points.at(ind).b = 255;
				}
				
				geoprob_cloud->points[ind] = inlier_cloud->points[ind];	
				geoprob_cloud->points[ind].r = 255.0*geoprob;
				geoprob_cloud->points[ind].g = 255.0*geoprob;
				geoprob_cloud->points[ind].b = 255.0*geoprob;
				
				prob_cloud->points[ind] = inlier_cloud->points[ind];
				prob_cloud->points[ind].r = 255.0*(geoprob-prob);
				prob_cloud->points[ind].g = 255.0*(geoprob-prob);
				prob_cloud->points[ind].b = 255.0*(geoprob-prob);

				colprob_cloud->points[ind] = inlier_cloud->points[ind];
				if(vec.size() > 1){
					float colprob = vec.at(1);
					colprob_cloud->points[ind].r = 255.0*colprob;
					colprob_cloud->points[ind].g = 255.0*colprob;
					colprob_cloud->points[ind].b = 255.0*colprob;
				}	
			}
		}
			
		showNr++;

		inlier_clouds.push_back(inlier_cloud);
		prob_clouds.push_back(prob_cloud);
		colprob_clouds.push_back(colprob_cloud);
		geoprob_clouds.push_back(geoprob_cloud);
		
		debuggdata.push_back(new SurfaceRefinementDebugg());
		debuggdata.back()->inliers	= inlier_cloud;
		debuggdata.back()->probs	= prob_cloud;
		debuggdata.back()->geoprobs	= geoprob_cloud;
		debuggdata.back()->colprobs = colprob_cloud;
		//bool save;
//		trackershow();
/*
			IplImage * prob_img = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 3);
			IplImage * mask_img = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 3);
			for(int w = 0; w < width; w++){
				for(int h = 0; h < height; h++){
					float p = probability[w][h];
					cvRectangle(prob_img,cvPoint(w, h),cvPoint(w, h),cvScalar(255.0*p,255.0*p,255.0*p, 0), 1 , 8, 0);
					if(mask[w][h]){	cvRectangle(mask_img,cvPoint(w, h),cvPoint(w, h),cvScalar(255.0,0,255.0, 0), 1 , 8, 0);}
					else{			cvRectangle(mask_img,cvPoint(w, h),cvPoint(w, h),cvScalar(b[w][h], g[w][h], r[w][h], 0), 1 , 8, 0);}
					if(z[w][h] == 0){cvRectangle(mask_img,cvPoint(w, h),cvPoint(w, h),cvScalar(0,255,0, 0), 1 , 8, 0);}
				}
			}
			
			//printf("HOW THE FUCK DID IT GET HERE?!\n");
			//exit(0);
			
			cvShowImage("prob_img", prob_img);
			cvShowImage("mask_img", mask_img);
			cvWaitKey(0);
			
			char buf[1024];
			sprintf(buf,"prob_img%i.png",improve_count);
			cvSaveImage(buf,prob_img);
			
			sprintf(buf,"mask_img%i.png",improve_count);
			cvSaveImage(buf,mask_img);
			
			//improve_count++;
			
			cvReleaseImage( &prob_img );
			cvReleaseImage( &mask_img );

*/
		}
		surface->update(px,py,pz,pweight);
		total_update_time += getCurrentTime()-start_update_time;
		improve_count++;
	/*
		if(false && (modelFactory->frame_nr > 49)){
			IplImage * prob_img = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 3);
			IplImage * mask_img = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 3);
			for(int w = 0; w < width; w++){
				for(int h = 0; h < height; h++){
					float p = probability[w][h];
					cvRectangle(prob_img,cvPoint(w, h),cvPoint(w, h),cvScalar(255.0*p,255.0*p,255.0*p, 0), 1 , 8, 0);
					if(mask[w][h]){	cvRectangle(mask_img,cvPoint(w, h),cvPoint(w, h),cvScalar(255.0,0,255.0, 0), 1 , 8, 0);}
					else{			cvRectangle(mask_img,cvPoint(w, h),cvPoint(w, h),cvScalar(b[w][h], g[w][h], r[w][h], 0), 1 , 8, 0);}
					//if(z[w][h] == 0){cvRectangle(mask_img,cvPoint(w, h),cvPoint(w, h),cvScalar(0,255,0, 0), 1 , 8, 0);}
				}
			}
			cvShowImage("prob_img", prob_img);
			cvShowImage("mask_img", mask_img);
			cvWaitKey(0);
			
			char buf[1024];
			sprintf(buf,"prob_img%i.png",improve_count);
			cvSaveImage(buf,prob_img);
			
			sprintf(buf,"mask_img%i.png",improve_count);
			cvSaveImage(buf,mask_img);
			
			improve_count++;
			
			cvReleaseImage( &prob_img );
			cvReleaseImage( &mask_img );

		}
	*/
		for(int i = 0; i < models->size(); i++){
			delete models->at(i);
		}
		delete models;
	}

	if(debugg){
		printf("---LOOP---\n");
		printf("distance_time        = %f\n",total_distance_time);
		printf("train_time           = %f\n",total_train_time);
		printf("inference_time       = %f\n",total_inference_time);
		printf("update_time          = %f\n",total_update_time);
		printf("-----\n");
		printf("total loop cost      = %f\n",total_distance_time+total_train_time+total_inference_time+total_update_time);
		printf("---END LOOP---\n");
	}
	
	if(debugg){printf("improve cost: %f\n",getCurrentTime()-start);}

	//Clear mem
	
	for(int w = 0; w < width; w++){
		delete[] distance[w];
		delete[] probability[w];
		
		delete[] mask[w];
	
		delete[] r[w];
		delete[] g[w];
		delete[] b[w];
		
		delete[] x[w];
		delete[] y[w];
		delete[] z[w];
		
		delete[] nx[w];
		delete[] ny[w];
		delete[] nz[w];
	}
	delete[] mask;
	
	delete[] distance;
	delete[] probability;
	
	delete[] r;
	delete[] g;
	delete[] b;
		
	delete[] x;
	delete[] y;
	delete[] z;
		
	delete[] nx;
	delete[] ny;
	
	modelFactory->setMaxDistance(modelFactory->getMaxDistance()+0.05);
	return cloud;
}
}
