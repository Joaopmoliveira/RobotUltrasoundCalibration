#ifndef _CALIBRATION_SOLVER_CPP
#define _CALIBRATION_SOLVER_CPP

#include <stdint.h>
 
void cpp_solver_calibration(size_t n_wires, size_t max_iters, size_t n_observations,const double* images, const double* transformations, double* solution);

#endif