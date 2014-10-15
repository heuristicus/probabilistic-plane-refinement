#include "AdaptiveSingleGaussiansSegmentModelFactory.h"
//#include "mygeometry/mygeometry.h"
#include <stdlib.h>
#include <sys/time.h>
#include <iostream>
#include <fstream>
//#include <pcl/visualization/pcl_plotter.h>
namespace ppr {
AdaptiveSingleGaussiansSegmentModelFactory::AdaptiveSingleGaussiansSegmentModelFactory(float md){
	max_d = md;
	smoothing = 50;
	bins = 500;
	max_bins = 500;
	min_bins = 10;
	data_per_bin = 500;

	smoothng_std = 0.001;
	P_enforcement = 5;
	
	for(int i = 0; i < bins; i++)			{x_arr[i] = 2.0f*max_d*(float(i)+0.5f-0.5*float(bins))/float(bins-1);}
	for(int i = 0; i < smoothing*2+1; i++)	{smoothing_kernel[i] = exp(-0.5*x_arr[i-smoothing+bins/2]*x_arr[i-smoothing+bins/2]/(smoothng_std*smoothng_std));}
	for(int i = 0; i < smoothing*2+1; i++)	{printf("%f\n",smoothing_kernel[i]);}
}

AdaptiveSingleGaussiansSegmentModelFactory::AdaptiveSingleGaussiansSegmentModelFactory(){
	max_d = 0.1f;
	smoothing = 50;
	bins = 500;
	max_bins = 500;
	min_bins = 10;
	data_per_bin = 500;
	smoothng_std = 0.001;
	P_enforcement = 5;

	for(int i = 0; i < bins; i++)			{x_arr[i] = 2.0f*max_d*(float(i)+0.5f-0.5*float(bins))/float(bins-1);}
	for(int i = 0; i < smoothing*2+1; i++)	{smoothing_kernel[i] = exp(-0.5*x_arr[i-smoothing+bins/2]*x_arr[i-smoothing+bins/2]/(smoothng_std*smoothng_std));}
	for(int i = 0; i < smoothing*2+1; i++)	{printf("%f\n",smoothing_kernel[i]);}
}

AdaptiveSingleGaussiansSegmentModelFactory::~AdaptiveSingleGaussiansSegmentModelFactory(){}

vector<SegmentModel*> * AdaptiveSingleGaussiansSegmentModelFactory::getModels(	vector<int>  w, vector<int>  h, vector<float> d, float ** x, float ** y, float ** z, float ** r, float ** g, float ** b,float ** nx, float ** ny, float ** nz,SegmentModel * base){
	vector<SegmentModel*> * ret = new vector<SegmentModel*>();

	if(d.size() > 5){
		int nr_bins = d.size()/data_per_bin;
		nr_bins = max(min(nr_bins,max_bins),min_bins);
		bins = nr_bins;
		//printf("good nr bins: %i\n",nr_bins);
		for(int i = 0; i < bins; i++)			{x_arr[i] = 2.0f*max_d*(float(i)+0.5f-0.5*float(bins))/float(bins-1);}
		for(int i = 0; i < smoothing*2+1; i++)	{smoothing_kernel[i] = 0;}
		for(int i = 0; i < smoothing*2+1; i++)	{
			int ind = i-smoothing+bins/2;
			if(ind >= 0 && ind < bins){
				double xarri = x_arr[ind];
				double inp = -0.5*xarri*xarri/(smoothng_std*smoothng_std);
				double sm = exp(inp);
				smoothing_kernel[i] = sm;
			}
		}
		
		//printf("max_d: %f\n",max_d);
		
		double * d_distr = new double[bins];
		double * d_distr_smooth = new double[bins];

		generate_d_distr(d_distr, w,h, d, z,  max_d);
		smooth_signal(d_distr_smooth, d_distr, smoothing_kernel);
		
		double d_sum = 0;
		double d_max = 0;
		for(int i = 0; i < bins; i++)				{
			d_sum += d_distr[i];
			d_max = max(d_max,d_distr[i]);
		}
		
		double smooth_d_sum = 0;
		double smooth_d_max = 0;
		for(int i = 0; i < bins; i++)				{
			smooth_d_sum += d_distr_smooth[i];
			smooth_d_max = max(smooth_d_max,d_distr_smooth[i]);
		}

		double inv_sdsum = 1.0/smooth_d_sum;
		double inv_dsum = 1.0/d_sum;
		for(int i = 0; i < bins; i++){
			//d_distr_smooth[i]*= inv_sdsum;
			//d_distr[i]*= inv_dsum;
		}

		vector<int> maximas_and_minimas = find_maximas_and_minimas(d_distr_smooth);

		//select area to optimize around
		int max_ind = -1;
		int max_vec_ind = -1;
		float max_val = -1;
		for(unsigned int i = 0; i < maximas_and_minimas.size(); i++){
			if(d_distr_smooth[maximas_and_minimas.at(i)] > max_val){
				max_val = d_distr_smooth[maximas_and_minimas.at(i)];
				max_ind = maximas_and_minimas.at(i);
				max_vec_ind = i;
			}
		}

		max_val *= 1;
		float small_std = 100000;
		float small_mu  = 0;

		int left_vec_ind = max_vec_ind-1;
		int left_ind = -1;
		
		double right_mu;
		double right_std;
		double left_mu;
		double left_std;
		
		//Left side
		if(max_ind!=0){
			while(left_ind == -1 && left_vec_ind >= 0){
				left_ind = maximas_and_minimas.at(left_vec_ind);
				int diff = fabs(left_ind-max_ind);
				if(diff > bins/20){
					break;
				}else{
					left_ind = -1;
					left_vec_ind -= 1;
				}
				
			}
			
			if(left_ind != -1){
				vector<float> values_left;
				values_left.resize(max_ind-left_ind+1);
				vector<float> x_left;
				x_left.resize(max_ind-left_ind+1);

				for(int i = left_ind; i <= max_ind; i++){
					x_left.at(i-left_ind) = x_arr[i];
					values_left.at(i-left_ind) = d_distr_smooth[i]/max_val;
				}
				std::pair <double,double> gaussian_left = fitGaussian(values_left, x_left);
				left_std = gaussian_left.first;
				left_mu  = gaussian_left.second;
				if(gaussian_left.first < small_std){
					small_std = gaussian_left.first;
					small_mu  = gaussian_left.second;
				}
			}
		}

		int right_vec_ind = max_vec_ind+1;
		int right_ind = -1;
		//Right side
		if(max_ind != bins-1){
			while(right_ind == -1 && right_vec_ind < maximas_and_minimas.size()){
				right_ind = maximas_and_minimas.at(right_vec_ind);
				int diff = fabs(right_ind-max_ind);
				if(diff > bins/20){
					break;
				}else{
					right_ind = -1;
					right_vec_ind += 1;
				}
			}
			if(right_ind != -1){		
				std::vector<float> values_right;
				std::vector<float> x_right;

				for(int i = max_ind; i < right_ind; i++){
					x_right.push_back(x_arr[i]);
					values_right.push_back(d_distr_smooth[i]/max_val);
				}

				std::pair <double,double> gaussian_right = fitGaussian(values_right, x_right);
				right_std = gaussian_right.first;
				right_mu  = gaussian_right.second;
				if(gaussian_right.first < small_std){
					small_std = gaussian_right.first;
					small_mu  = gaussian_right.second;
				}
			}
		}

		ret->push_back(new GaussiansSegmentModel(max_val, small_mu, small_std, max_d,bins, d_distr_smooth));

		expectedInliers = max_val*sqrt(2.0*M_PI*small_std*small_std)/(x_arr[1]-x_arr[0]);
		
		max_d = small_std*6;

		if(debugg){
			double mean_whole = 0;
			double total = 0;
			for(int i = 0; i < d.size(); i++){mean_whole += d.at(i);}
			mean_whole /= float(d.size());
			
			float std_whole = 0;
			for(int i = 0; i < d.size(); i++){std_whole += (d.at(i)-mean_whole)*(d.at(i)-mean_whole);}
			std_whole /= float(d.size()-1);
			std_whole = sqrt(std_whole);
			
			char fit_whole_buf[500000];
			sprintf(fit_whole_buf,"fit_whole = [ ");
			for(int i = 0; i < bins; i++){
				double x = x_arr[i] - mean_whole;
				double e = max_val*exp(-0.5*x*x/(std_whole*std_whole));
				sprintf(fit_whole_buf,"%s %f ",fit_whole_buf,e);
			}
			sprintf(fit_whole_buf,"%s]",fit_whole_buf);
			
			
			//Calculate gaussian over monotonicly decreasing parts
			std::vector<float> values_both;
			std::vector<float> x_both;

			for(int i = left_ind; i < right_ind; i++){
				x_both.push_back(x_arr[i]);
				values_both.push_back(d_distr_smooth[i]/max_val);
			}
			std::pair <double,double> gaussian_both = fitGaussian(values_both, x_both, x_arr[max_ind]);
			float both_std = gaussian_both.first;
			float both_mu  = gaussian_both.second;
			
			char fit_both_buf[500000];
			sprintf(fit_both_buf,"fit_both = [ ");
			for(int i = 0; i < bins; i++){
				double x = x_arr[i] - both_mu;
				double e = max_val*exp(-0.5*x*x/(both_std*both_std));
				sprintf(fit_both_buf,"%s %f ",fit_both_buf,e);
			}
			sprintf(fit_both_buf,"%s]",fit_both_buf);
			
			//Calculate gaussian for both sides and pick the best solution
			
			char fit_left_buf[500000];
			sprintf(fit_left_buf,"fit_left = [ ");
			for(int i = 0; i < bins; i++){
				double x = x_arr[i] - both_mu;
				double e = max_val*exp(-0.5*x*x/(left_std*left_std));
				sprintf(fit_left_buf,"%s %f ",fit_left_buf,e);
			}
			sprintf(fit_left_buf,"%s]",fit_left_buf);
			
			char fit_right_buf[500000];
			sprintf(fit_right_buf,"fit_right = [ ");
			for(int i = 0; i < bins; i++){
				double x = x_arr[i] - both_mu;
				double e = max_val*exp(-0.5*x*x/(right_std*right_std));
				sprintf(fit_right_buf,"%s %f ",fit_right_buf,e);
			}
			sprintf(fit_right_buf,"%s]",fit_right_buf);
		
			char x_arr_buf[500000];
			char d_distr_buf[500000];
			char d_distr_smooth_buf[500000];
			char fit_buf[500000];

			sprintf(x_arr_buf,"x_arr = [ ");
			sprintf(d_distr_buf,"d_distr = [ ");
			sprintf(d_distr_smooth_buf,"d_distr_smooth = [ ");
			sprintf(fit_buf,"fit = [ ");

			for(int i = 0; i < bins; i++){
				sprintf(x_arr_buf,"%s %f ",x_arr_buf, x_arr[i]);
				sprintf(d_distr_buf,"%s %f ",d_distr_buf,d_distr[i]);
				sprintf(d_distr_smooth_buf,"%s %f ",d_distr_smooth_buf,d_distr_smooth[i]);

				double x = x_arr[i] - small_mu;
				double e = max_val*exp(-0.5*x*x/(small_std*small_std));
				sprintf(fit_buf,"%s %f ",fit_buf,e);
			}
			sprintf(x_arr_buf,"%s]",x_arr_buf);
			sprintf(d_distr_buf,"%s]",d_distr_buf);
			sprintf(d_distr_smooth_buf,"%s]",d_distr_smooth_buf);
			sprintf(fit_buf,"%s]",fit_buf);

			char prob_fit_buf		[500000];
			char prob_fit_whole_buf	[500000];
			char prob_fit_both_buf	[500000];
			char prob_fit_right_buf	[500000];
			char prob_fit_left_buf	[500000];
			
			sprintf(prob_fit_buf,			"prob_fit = [ ");
			sprintf(prob_fit_whole_buf,		"prob_fit_whole = [ ");
			sprintf(prob_fit_both_buf,		"prob_fit_both = [ ");
			sprintf(prob_fit_right_buf,		"prob_fit_right = [ ");
			sprintf(prob_fit_left_buf,		"prob_fit_left = [ ");
			
			for(int i = 0; i < bins; i++){
			
				double x = x_arr[i] - small_mu;
				double G = max_val*exp(-0.5*x*x/(small_std*small_std));
				double H = max(max_val*0.05,d_distr_smooth[i]);
				sprintf(prob_fit_buf,"%s %f ",prob_fit_buf,min(G/H,1.0));

				x = x_arr[i] - mean_whole;
				G = max_val*exp(-0.5*x*x/(std_whole*std_whole));
				H = max(max_val*0.005,d_distr_smooth[i]);
				sprintf(prob_fit_whole_buf,"%s %f ",prob_fit_whole_buf,min(G/H,1.0));
				
				x = x_arr[i] - both_mu;
				G = max_val*exp(-0.5*x*x/(both_std*both_std));
				H = max(max_val*0.005,d_distr_smooth[i]);
				sprintf(prob_fit_both_buf,"%s %f ",prob_fit_both_buf,min(G/H,1.0));
				
				x = x_arr[i] - left_mu;
				G = max_val*exp(-0.5*x*x/(left_std*left_std));
				H = max(max_val*0.005,d_distr_smooth[i]);
				sprintf(prob_fit_left_buf,"%s %f ",prob_fit_left_buf,min(G/H,1.0));
				
				x = x_arr[i] - right_mu;
				G = max_val*exp(-0.5*x*x/(right_std*right_std));
				H = max(max_val*0.005,d_distr_smooth[i]);
				sprintf(prob_fit_right_buf,"%s %f ",prob_fit_right_buf,min(G/H,1.0));
			}
			sprintf(prob_fit_buf,"%s]",prob_fit_buf);
			sprintf(prob_fit_whole_buf,"%s]",prob_fit_whole_buf);
			sprintf(prob_fit_both_buf,"%s]",prob_fit_both_buf);
			sprintf(prob_fit_left_buf,"%s]",prob_fit_left_buf);
			sprintf(prob_fit_right_buf,"%s]",prob_fit_right_buf);


			//string b = "octave --eval \"";
			string b = "/usr/local/MATLAB/R2014a/bin/matlab -nosplash -nodesktop -r \"";
			string matlabfile = "";
			matlabfile 		+= "cd /home/johane/catkin_ws;";

			matlabfile		+=string(x_arr_buf)+";";
			matlabfile		+=string(d_distr_buf)+";";
			matlabfile		+=string(d_distr_smooth_buf)+";";

			matlabfile		+=string(fit_buf)+";";
			matlabfile		+=string(fit_whole_buf)+";";
			matlabfile		+=string(fit_both_buf)+";";
			matlabfile		+=string(fit_right_buf)+";";
			matlabfile		+=string(fit_left_buf)+";";
			
			matlabfile		+=string(prob_fit_buf)+";";
			matlabfile		+=string(prob_fit_whole_buf)+";";
			matlabfile		+=string(prob_fit_both_buf)+";";
			matlabfile		+=string(prob_fit_right_buf)+";";
			matlabfile		+=string(prob_fit_left_buf)+";";
			
//b		+="\n";

			string f1 = "";
			f1		+="f1 = figure();";
			f1		+="hold on;";
			f1		+="title('Histogram');";
			f1		+="xlabel('Distance to surface');";
			f1		+="ylabel('Fraction of measurements');";
			f1		+="plot(x_arr,d_distr,'-r','LineWidth',2.0);";
			f1		+="plot(x_arr,d_distr_smooth,'-g','LineWidth',2.0);";
			f1		+="legend('H pre smoothing','H post smoothing');";
			char fnbuf[1024];
			sprintf(fnbuf,"histogram_frame%.4i_iteration%.3i",frame_nr,iteration_nr);
			f1		+="print('-f1', '-r600', '-depsc', '"+string(fnbuf)+"');";
			//printf("f1: %s\n",f1.c_str());
			matlabfile += f1;

//b		+="\n";

			string f2 = "";
			f2		+="f2 = figure();";
			f2		+="hold on;";
			f2		+="title('Histogram and fitted functions for G');";
			f2		+="xlabel('Distance to surface');";
			f2		+="ylabel('Number of measurements');";
			f2		+="plot(x_arr,d_distr_smooth,'-g','LineWidth',2.0);";
			f2		+="plot(x_arr,fit_whole,'-r','LineWidth',2.0);";
			f2		+="plot(x_arr,fit_both,'-b','LineWidth',2.0);";
			f2		+="plot(x_arr,fit_left,'-m','LineWidth',2.0);";
			f2		+="plot(x_arr,fit_right,'-c','LineWidth',2.0);";
			char tmp_buf[1024];
			sprintf(tmp_buf,"plot([%f %f],[%f %f],'xr','LineWidth',3.0);",x_arr[left_ind],x_arr[right_ind],d_distr_smooth[left_ind],d_distr_smooth[right_ind]);
			f2		+= string(tmp_buf);
			//f2		+="plot(x_arr,fit_right,'-c','LineWidth',2.0);";
			f2		+="legend('H', 'G naive', 'G decreasing', 'G left', 'G right','Borders');";
			sprintf(fnbuf,"fit_frame%.4i_iteration%.3i",frame_nr,iteration_nr);
			f2		+="print('-f2', '-r600', '-depsc', '"+string(fnbuf)+"');";
			//printf("f2: %s\n",f2.c_str());
			matlabfile += f2;

//b		+="\n";

			string f3 = "";
			f3		+="f3 = figure();";
			//f3		+="axis([-0.04 0.04 0 1.05])";
			f3		+="hold on;";
			f3		+="title('Histogram and inlier probability G/H');";
			f3		+="xlabel('Distance to surface');";
			f3		+="ylabel('inlier probability');";
			f3		+="plot(x_arr,d_distr_smooth./max(d_distr_smooth),'-g','LineWidth',2.0);";
			f3		+="plot(x_arr,prob_fit,'-r','LineWidth',2.0);";
			f3		+="plot(x_arr,fit./max(d_distr_smooth),'-b','LineWidth',2.0);";
			f3		+="legend('H', 'G/H', 'G');";
			sprintf(fnbuf,"prob_frame%.4i_iteration%.3i",frame_nr,iteration_nr);
			f3		+="print('-f3', '-r600', '-depsc', '"+string(fnbuf)+"');";
			matlabfile += f3;

			//b		+="exit \"";
			b+= matlabfile;
			
			
			ofstream myfile;
			sprintf(fnbuf,"matlabfile_f%.4i_i%.3i.m",frame_nr,iteration_nr);
			myfile.open (fnbuf);
			myfile << matlabfile;
			myfile.close();
			b		+="\"";
			//printf("adaptive before system call\n");
			
			//printf("%s\n",b.c_str());
			//system(b.c_str());
			//system("/usr/local/MATLAB/R2014a/bin/matlab");
			//system("ls -al");
			//printf("adaptive after system call\n");
			//exit(0);
		}

	}
	return ret;
}

float AdaptiveSingleGaussiansSegmentModelFactory::getExpectedInliers(){
	return expectedInliers;
}

float AdaptiveSingleGaussiansSegmentModelFactory::getMaxDistance(){
	return max_d;
}

void AdaptiveSingleGaussiansSegmentModelFactory::setMaxDistance(float f){
	//printf("%f %f\n",f,max_d);
	max_d = f;
}

vector<int> AdaptiveSingleGaussiansSegmentModelFactory::find_maximas_and_minimas(double * d_distr_smooth){
	vector<int> maximas_and_minimas;
	maximas_and_minimas.push_back(0);
	for(int i = 1; i < bins-1; i++){
		float prev = d_distr_smooth[i-1];
		float current = d_distr_smooth[i];
		float next = d_distr_smooth[i+1];
		if(((current >= prev) && (current > next)) || ((current < prev) && (current <= next))) {maximas_and_minimas.push_back(i);}
	}
	maximas_and_minimas.push_back(bins-1);
	return maximas_and_minimas;
}

void AdaptiveSingleGaussiansSegmentModelFactory::generate_d_distr(double * d_distr, vector<int> & v_w, vector<int> & v_h, vector<float> & v_d, float ** z, float max_d ){
	for(int i = 0; i < bins; i++)				{d_distr[i] = 0;}

	int v_d_size = v_d.size();
	float mul = bins/(2*max_d);
	
	//omp_set_dynamic(0);
	//#pragma omp parallel num_threads(7)
	{
		float tmp[bins];
		for(int i = 0; i < bins; i++){tmp[i] = 0;}
		//#pragma omp for
		for(unsigned int i = 0; i < v_d_size; i++){
			float d = v_d.at(i);
			if(fabs(d) >= max_d){continue;}
			int ind = int((d+max_d)*mul);
			tmp[ind]++;
		}
		
		//#pragma omp critical
		{
			for(int i = 0; i < bins; i++){
				d_distr[i]+=tmp[i];
			}
		}
	}
	//double inv_vds = 1.0/double(v_d_size);
	//for(int i = 0; i < bins; i++)				{d_distr[i] *= inv_vds;}
}

void AdaptiveSingleGaussiansSegmentModelFactory::smooth_signal(double * d_distr_smooth, double * d_distr, double * smoothing_kernel){
	for(int i = 0; i < bins; i++){d_distr_smooth[i] = 0;}
	double tota_sum = 0;

	for(int i = 0; i < bins; i++){
		double sum = 0;
		double normalizer = 0;
		for(int j = -smoothing; j <= smoothing; j++){
			if((i+j)>=0 && (i+j)<bins){
				int current_ind = j+smoothing;
				
				double mul = smoothing_kernel[current_ind];
				normalizer += mul;
				sum += d_distr[i+j]*mul;
			}
		}
		d_distr_smooth[i] = sum/normalizer;
		tota_sum = max(tota_sum,d_distr_smooth[i]);
	}
}

std::pair <double,double> AdaptiveSingleGaussiansSegmentModelFactory::fitGaussian(std::vector<float> value, std::vector<float> index){
	int iter = 25;
	double h = 0.000000001;

	double mu_mid = index.back();if(value.front() > value.back()){mu_mid = index.front();}
	double sigma_mid = 0;

	for(unsigned int i = 0; i < value.size(); i++){sigma_mid += (index.at(i)-mu_mid)*(index.at(i)-mu_mid)*fabs(value.at(i));}sigma_mid = sqrt(sigma_mid);
	double sigma_max = sigma_mid*2;
	double sigma_min = 0;
	double mu_max = mu_mid+0.005;
	double mu_min = mu_mid-0.005;
	for(int i = 0; i < iter; i++){
		sigma_mid = (sigma_max+sigma_min)/2;
		double sigma_neg = gausian_error(value,index, sigma_mid-h,mu_mid);
		double sigma_pos = gausian_error(value,index, sigma_mid+h,mu_mid);
		if(sigma_neg < sigma_pos){sigma_max = sigma_mid;}else{sigma_min = sigma_mid;}
	}
	sigma_mid = (sigma_max+sigma_min)/2;

	return std::make_pair(sigma_mid,mu_mid);
}

std::pair <double,double> AdaptiveSingleGaussiansSegmentModelFactory::fitGaussian(std::vector<float> value, std::vector<float> index, float mu_mid){
	int iter = 25;
	double h = 0.000000001;

	double sigma_mid = 0;

	for(unsigned int i = 0; i < value.size(); i++){sigma_mid += (index.at(i)-mu_mid)*(index.at(i)-mu_mid)*fabs(value.at(i));}sigma_mid = sqrt(sigma_mid);
	double sigma_max = sigma_mid*2;
	double sigma_min = 0;

	for(int i = 0; i < iter; i++){
		sigma_mid = (sigma_max+sigma_min)/2;
		double sigma_neg = gausian_error(value,index, sigma_mid-h,mu_mid);
		double sigma_pos = gausian_error(value,index, sigma_mid+h,mu_mid);
		if(sigma_neg < sigma_pos){sigma_max = sigma_mid;}else{sigma_min = sigma_mid;}
	}
	sigma_mid = (sigma_max+sigma_min)/2;

	return std::make_pair(sigma_mid,mu_mid);
}

double AdaptiveSingleGaussiansSegmentModelFactory::gausian_error(std::vector<float> value, std::vector<float> index, double sigma, double mu){
	double error = 0;
	unsigned int nr_values = value.size();
	double isigma2 = 1.0f/(sigma*sigma);
	for(unsigned int i = 0; i < nr_values; i++){
		double x = index.at(i) - mu;
		double e = value.at(i) - exp(-0.5*x*x*isigma2);
		if(e < 0){e*=P_enforcement;}
		error += e*e;
	}
	return error;	
}
}
