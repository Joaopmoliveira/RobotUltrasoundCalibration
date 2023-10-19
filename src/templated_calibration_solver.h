#ifndef _TEMPLATED_CALIBRATION_SOLVER_CPP
#define _TEMPLATED_CALIBRATION_SOLVER_CPP

#include <stdint.h>

template<size_t n_wires>
void templated_solver_calibration(size_t max_iters, size_t n_observations, const double* images, const double* transformations, double* solution) {
constexpr size_t number_of_strings = n_wires;
constexpr size_t number_of_variables = 6 + 4 * number_of_strings;
double variables[number_of_variables];

for (auto& val : variables)
val = 0.0;

ceres::Problem problem;
for (size_t observ = 0; observ < n_observations; ++observ,transformations+=3*4){
const double *transformations_pointer = transformations;
Observation observation;
observation.flange_configuration.values[0] = *(transformations_pointer + 0);
observation.flange_configuration.values[1] = *(transformations_pointer + 1);
observation.flange_configuration.values[2] = *(transformations_pointer + 2);

observation.flange_configuration.values[3] = *(transformations_pointer + 3);
observation.flange_configuration.values[4] = *(transformations_pointer + 4);
observation.flange_configuration.values[5] = *(transformations_pointer + 5);

observation.flange_configuration.values[6] = *(transformations_pointer + 6);
observation.flange_configuration.values[7] = *(transformations_pointer + 7);
observation.flange_configuration.values[8] = *(transformations_pointer + 8);

observation.flange_configuration.values[9] = *(transformations_pointer + 9);
observation.flange_configuration.values[10] = *(transformations_pointer + 10);
observation.flange_configuration.values[11] = *(transformations_pointer + 11);

for (size_t wire_number = 0; wire_number < number_of_strings; ++wire_number, images+=3){
const double *image_pointer = images;
observation.wire_number = wire_number;
observation.wire_data.values[0] = *(image_pointer + 0);
observation.wire_data.values[1] = *(image_pointer + 1);
observation.wire_data.values[2] = *(image_pointer + 2);

ceres::CostFunction *cost_function =
new ceres::AutoDiffCostFunction<WiredResidual, 2, number_of_variables>(
new WiredResidual(observation));
problem.AddResidualBlock(cost_function, nullptr, variables);
}
}

ceres::Solver::Options options;
options.max_num_iterations = max_iters;
options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = false;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    size_t iter = 0;
    for (auto& val : variables){
        solution[iter] = val;
        ++iter;
    }
    return;
}

#endif