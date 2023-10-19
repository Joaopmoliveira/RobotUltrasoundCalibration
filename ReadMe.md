## Robotic Ultrasound Calibration

This document introduces our implementation of a novel ultrasound calibration algorithm for robotic systems. We've designed this code in C++ to optimize solver speed while making it available to the broader scientific community for potential improvements.

Additionally, we've developed a MATLAB wrapper through a MEX function to simplify prototyping for both commercial and non-commercial use. The function's API is straightforward:

```matlab
% ImageData array of size [3 n_wires n_observations] where each column represents the position of the wire in the image frame 
% traditionally this would be extracted from the ultrasound scanner attached to the flange of the robot

% TransformData array of size [3 4 n_observations] where each 3x4 matrix corresponds to the sub-matrix of the homogenenous transformation
% of the flange to the base of the robot, i.e. T = [R p]   thus each stacked transform is just T_stack = [R p]
%                                                  [0 1]

Solution=CalibrationSolver(ImageData,TransformData);

% The solution is a vector of size 6+n_wires*4. Usually only the homogenenous transformation between the flange and the origin of the 
% ultrasound image is required, thus you just need to query the first 6 elements of the solution (the remaining ones correspond to the 
% estimated pose of the wires as observed from the base frame). 

% translation_image_to_flange = solution(4:6);
% euler_angles_image_to_flange = solution(1:3);  
```

Please note that the solver doesn't handle observations with substantial noise. You may need to implement an outer loop, such as a RANSAC approach, to address this issue.

## Build and Dependencies

Our code has one major dependency, the Ceres solver. To include it, follow these steps:

Install [vcpkg](https://vcpkg.io/en/) on your local machine.

Navigate to your vcpkg directory and execute the following command:

``` bash
vcpkg install ceres:x64-windows-static-md
```

>> Note: The 'md' flag is necessary because MATLAB uses dynamic linking, so we must also use dynamic linking for proper MEX code linking.

To compile the library source code, perform the following steps:

```bash
git clone 
cd RobotUltrasoundCalibration

cmake . -DCMAKE_TOOLCHAIN_FILE="~path_to_vcpkg_direcoty\scripts\buildsystems\vcpkg.cmake" -DVCPKG_TARGET_TRIPLET=x64-windows-static-md -DCMAKE_BUILD_TYPE=Release -G "Ninja"

cmake --build .
```

These instructions are aimed at Windows, but the library should work on systems that support Ceres. After compiling, you can copy the CalibrationSolver.mexw64 file to the desired directory for MATLAB function calls. To use the library in C++ projects, link your project to the calibration_solver target in CMake.

## Known Limitations

The authors of Ceres provide clean APIs to manipulate and deal with hogomogeneous transformations which facilitate the Auto-Differentations funtionality through the Jet class (if you have not seen this and you are a C++ programmer, please see the source code of Jet, its a marvel to look at). We implemented the homogenenous transformations by hand, thus the source code could be significantly simplified if this alternative API was used. 

## Citation

This library was developed to showcase our work to the scientific community. If you find this work useful in your own research, please cite our paper.


