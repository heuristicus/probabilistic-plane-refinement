#include "Surface.h"
using namespace std;
namespace ppr {
Surface::Surface(){}
Surface::~Surface(){}
float Surface::distance(float x, float y, float z){printf("distance not impelented for this surface\n");return -1;}
float Surface::angle(float x, float y, float z, float nx, float ny, float nz){printf("angle not impelented for this surface\n");return 1;}
void Surface::update(vector<float> & px, vector<float> & py, vector<float> & pz, vector<float> & pweight){printf("update not impelented for this surface\n");}
void Surface::print(){							printf("print not impelented for this surface\n");}
Surface * Surface::clone(){						printf("clone not impelented for this surface\n"); return 0;}
void Surface::transform(Eigen::Matrix4f t){		printf("transform not impelented for this surface\n");}
}
