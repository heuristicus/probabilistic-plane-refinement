#include "ColorHistogramSegmentModelFactory.h"
//#include "mygeometry/mygeometry.h"
#include <sys/time.h>
//#include <pcl/visualization/pcl_plotter.h>
#include <opencv.hpp>
namespace ppr {

ColorHistogramSegmentModelFactory::ColorHistogramSegmentModelFactory(){

	int h_res = 5;
	int s_res = 5;
	int v_res = 5;

	color_res = h_res*h_res*v_res;

	IplImage * rgb_img	= cvCreateImage(cvSize(256*256*256, 1), IPL_DEPTH_8U, 3);
	char * rgb_data		= (char *)rgb_img->imageData;

	for(int r = 0; r < 256; r++){
		for(int g = 0; g < 256; g++){
			for(int b = 0; b < 256; b++){
				int ind = r*255*255+g*255+b;
				rgb_data[3*ind+0] = r;
				rgb_data[3*ind+1] = g;
				rgb_data[3*ind+2] = b;
			}
		}
	}

	IplImage * hsv_img = cvCreateImage(cvGetSize(rgb_img), IPL_DEPTH_8U, 3);
	char * hsv_data		= (char *)hsv_img->imageData;
	cvCvtColor(rgb_img, hsv_img, CV_RGB2HLS);
	//cvCopy( rgb_img, hsv_img, NULL );

	int max_h = int(hsv_data[0]);
	int min_h = int(hsv_data[0]);

	int max_s = int(hsv_data[1]);
	int min_s = int(hsv_data[1]);

	int max_v = int(hsv_data[2]);
	int min_v = int(hsv_data[2]);
	for(int r = 0; r < 256; r++){
		for(int g = 0; g < 256; g++){
			for(int b = 0; b < 256; b++){
				int ind = r*255*255+g*255+b;
				int h = int(hsv_data[3*ind+0]);
				int s = int(hsv_data[3*ind+1]);
				int v = int(hsv_data[3*ind+2]);
				if(h > max_h){max_h = h;}
				if(h < min_h){min_h = h;}

				if(s > max_s){max_s = s;}
				if(s < min_s){min_s = s;}

				if(v > max_v){max_v = v;}
				if(v < min_v){min_v = v;}
				//printf("r: %i g: %i b:%i h: %i s:%i v:%i\n",r,g,b,h,s,v);
			}
		}
	}

	color_bin_index = new int[256*256*256];
	//printf("max and mins: %i %i, %i %i, %i %i\n", max_h,min_h,max_s,min_s,max_v,min_v);
	float mul_h = float(h_res) / float(max_h-min_h+1);
	float mul_s = float(s_res) / float(max_s-min_s+1);
	float mul_v = float(v_res) / float(max_v-min_v+1);
	for(int r = 0; r < 256; r++){
		for(int g = 0; g < 256; g++){
			for(int b = 0; b < 256; b++){
				int ind = r*255*255+g*255+b;
				int h = int(hsv_data[3*ind+0]);
				int s = int(hsv_data[3*ind+1]);
				int v = int(hsv_data[3*ind+2]);
				int h_bin = int(float(h-min_h)*mul_h);
				int s_bin = int(float(s-min_s)*mul_s);
				int v_bin = int(float(v-min_v)*mul_v);

				color_bin_index[r*255*255+g*255+b] = s_res*v_res*h_bin + v_res*s_bin + v_bin;
			}
		}
	}
	
	cvReleaseImage( &rgb_img );
	cvReleaseImage( &hsv_img );
	color_hist_weight = new float[color_res];
	color_hist_count = new float[color_res];
}
ColorHistogramSegmentModelFactory::~ColorHistogramSegmentModelFactory(){
	delete[] color_bin_index;
	delete[] color_hist_weight;
	delete[] color_hist_count;
}
vector<SegmentModel*> * ColorHistogramSegmentModelFactory::getModels(	vector<int>  w, vector<int>  h, vector<float> d, float ** x, float ** y, float ** z, float ** r, float ** g, float ** b,float ** nx, float ** ny, float ** nz,SegmentModel * base){
	vector<SegmentModel*> * ret = new vector<SegmentModel*>();

	for(int i = 0; i < color_res; i++){
		color_hist_weight[i] = 0;
		color_hist_count[i] = 0;
	}

	unsigned int w_size = w.size();
	
	for(unsigned int k = 0; k < w_size; k++){
		int i = w.at(k);
		int j = h.at(k);

		float r_k = r[i][j];
		float g_k = g[i][j];
		float b_k = b[i][j];
		
		int color_index = color_bin_index[255*255*int(r_k)+255*int(g_k)+int(b_k)];

		color_hist_weight[color_index]+=base->getModelFit(d.at(k),x[i][j],y[i][j],z[i][j],r_k,g_k,b_k,nx[i][j],ny[i][j],nz[i][j]);
		color_hist_count[color_index]++;
	}

	for(int i = 0; i < color_res; i++){
		color_hist_weight[i] /= max(color_hist_count[i],5.0f);
	}

	ret->push_back(new ColorHistogramSegmentModel(color_bin_index, color_hist_weight));
	return ret;
}
}
