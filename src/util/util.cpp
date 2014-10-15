#include "util.h"

#include <sys/time.h>
#include <cstddef>
namespace ppr {

double getCurrentTime(){
	struct timeval start;
	gettimeofday(&start, NULL);
	double sec = start.tv_sec;
	double usec = start.tv_usec;
	double t = sec+usec*0.000001;
	return t;
}

}
