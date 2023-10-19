## Robotic Ultrasound Calibration

The present document showcases our implementation of the novel ultrasound calibration algorithm for robotic systems. We designed the code in C++ to improve the speed of the solver whilst AND provide the broader scientific community with our implementation in the hopes of obtaining further improvements. 

We also designed a wrapper for matlab, through a MEX function to simplify prototyping further downstream solutions both for commercial and non-commercial use. The API of the call is fairly simplistic

```matlab
% ImageData array of size [3 n_wires n_observations] where each column represents the position of the wire in the image frame 
% traditionally this would be extracted from the ultrasound scanner attached to the flange of the robot

% TransformData array of size [3 4 n_observations] where each 3x4 matrix corresponds to the sub-matrix of the homogenenous transformation
% of the flange to the base of the robot, i.e. T = [R p]   thus each stacked transform is just T_stack = [R p]
%                                                  [0 1]

Solution=CalibrationSolver(ImageData,TransformData);
```

The underlying solver currently makes no attempts to deal with observations with large noise, thus you must either implement an outer loop
through a RANSAC approach or something simillar.

## Build and Dependencies

We have one major dependency in our code. The Ceres solver. To include it first install [vcpkg](https://vcpkg.io/en/) in your local machine.Once you have vcpkg please go to the directory of vcpkg and run the following command 

```
vcpkg install ceres:x64-windows-static-md
```

*note : the md flang is required because in windows you can have dynamic or static linking to the c-library, and because matlab uses dynamic linking, then we must also use dynamic linking to properly compile the MEX code

To compile the source code of the library please do the following. 

```
git clone 
cd RobotUltrasoundCalibration

cmake . -DCMAKE_TOOLCHAIN_FILE="~path_to_vcpkg_direcoty\scripts\buildsystems\vcpkg.cmake" -DVCPKG_TARGET_TRIPLET=x64-windows-static-md -DCMAKE_BUILD_TYPE=Release -G "Ninja"

cmake --build .
```

Notice that although these instructions are targeted at windows, the library itself should mostly work on any system which supports Ceres. Once this is done you can copy the CalibrationSolver.mexw64 file into any directory you wish the call the function from in Matlab. To use the library in your C++ projects, you can link your project to the calibration_solver target in CMake. 

## Citation

This library was developed to showcase to the broader scientific community our paper, thus if you find this work noteworthy in your own scientific travels please cite us through


