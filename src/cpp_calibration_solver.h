#ifndef _CALIBRATION_SOLVER_CPP
#define _CALIBRATION_SOLVER_CPP

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void cpp_solver_calibration(int n_wires, int max_iters, int n_observations,const double* images, const double* transformations, double* solution);

#ifdef __cplusplus
}
#endif

#endif