#include "cpp_calibration_solver.h"
#include "templated_calibration_solver.h"
#include <cstdint>
#include "wire_mathematics.h"
#include "ceres/ceres.h"

constexpr size_t one_wire = 1;
constexpr size_t two_wires = 2;
constexpr size_t three_wires = 3;
constexpr size_t four_wires = 4;
constexpr size_t five_wires = 5;
constexpr size_t six_wires = 6;

void cpp_solver_calibration(int n_wires, int max_iters, int n_observations,const double* images, const double* transformations,double* initial_guess, double* solution){
	switch (n_wires) {
	case 0:
		return;
		break;
	case one_wire:
		templated_solver_calibration<one_wire>(max_iters,n_observations,images,transformations,initial_guess,solution);
		break;
	case two_wires:
		templated_solver_calibration<two_wires>(max_iters, n_observations, images, transformations,initial_guess, solution);
		break;
	case three_wires:
		templated_solver_calibration<three_wires>(max_iters, n_observations, images, transformations,initial_guess, solution);
		break;
	case four_wires:
		templated_solver_calibration<four_wires>(max_iters, n_observations, images, transformations,initial_guess, solution);
		break;
	case five_wires:
		templated_solver_calibration<five_wires>(max_iters, n_observations, images, transformations,initial_guess, solution);
		break;
	case six_wires:
		templated_solver_calibration<six_wires>(max_iters, n_observations, images, transformations,initial_guess, solution);
		break;
	default:
		return;
		break;
	}
	return;
}
