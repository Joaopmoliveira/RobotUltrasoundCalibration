/*
MEX implementation of the imread function from openCV.

cmake ..\CppWrapper -DCMAKE_TOOLCHAIN_FILE="C:\Dev\vcpkg\scripts\buildsystems\vcpkg.cmake" -DVCPKG_TARGET_TRIPLET=x64-windows-static-md -DCMAKE_BUILD_TYPE=Release -G "Ninja"

*/

#include <stdio.h>
#include <string.h>
#include "mex.h"
#include "matrix.h"
#include "cpp_calibration_solver.h"

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]){

    //checking inputs/outputs
    if (nrhs < 2 )
        mexErrMsgTxt("The function takes at minimum two arguments");

    if (nrhs > 4)
        mexErrMsgTxt("The function takes at maximum four arguments");

    int n_iterations = 1000;
    if (nrhs == 3){
        if(!mxIsInt32(prhs[2]) || mxGetNumberOfElements(prhs[2]) != 1){
            mexErrMsgTxt("The function takes the maximum number of iterations as the third parameter which must be an integer");
        }
         
        void* local = mxGetData(prhs[2]);
        n_iterations = *((int*)local);
    }

    if (nlhs != 1)
        mexErrMsgTxt("A single output is returned");

    // check if input is matrix with proper size

    // image data
    if(mxIsSparse(prhs[1]) ||
       mxIsComplex(prhs[1]) ||
       mxIsClass(prhs[1],"char")) {
        mexErrMsgIdAndTxt("MyToolbox:myfcn:nrhs",
                          "second input must be full matrix of real values.");
    }

    // transform data
    if(mxIsSparse(prhs[0]) ||
       mxIsComplex(prhs[0]) ||
       mxIsClass(prhs[0],"char")) {
        mexErrMsgIdAndTxt("MyToolbox:myfcn:nrhs",
                          "first input must be full matrix of real values.");
    }

    const mwSize dimensions_image = mxGetNumberOfDimensions(prhs[0]);
    const mwSize dimensions_homogenenous = mxGetNumberOfDimensions(prhs[1]);

    if(dimensions_image!=3 ){
        mexErrMsgIdAndTxt("MyToolbox:myfcn:nrhs",
                          "The dimensions of the first input must be [3 n_wires n_observations]");
    }

    if(dimensions_homogenenous!=3 ){
        mexErrMsgIdAndTxt("MyToolbox:myfcn:nrhs",
                          "The dimensions of the second input must be [3 4 n_observations]");
    }

    const mwSize *inDimImage = mxGetDimensions(prhs[0]);
    const mwSize *inDimHomogenenous = mxGetDimensions(prhs[1]);

    // check image data first
    if(inDimImage[0]!= 3 ){
        mexErrMsgIdAndTxt("MyToolbox:myfcn:nrhs",
                          "The input data must have size(input,1) = 3");
    }

    if(inDimImage[1]<1 || inDimImage[1]>6)
        mexErrMsgIdAndTxt("MyToolbox:myfcn:nrhs",
                          "The input data can only have up to six wires, i.e. n_wires in [1 ... 6]");

    // check transform data now
    if(inDimHomogenenous[0]!=3 && inDimHomogenenous[1]!=4)
        mexErrMsgIdAndTxt("MyToolbox:myfcn:nrhs",
                          "The homogenenous transformations must have a size of [3 x 4 x n_observations]");

    // check that the number of observations of both data types is the same
    if(inDimHomogenenous[2]!=inDimImage[2])
        mexErrMsgIdAndTxt("MyToolbox:myfcn:nrhs",
                          "The number of observations in the image data and transform data must be the same");

    //convert first input to double vector
    double* inImageData = mxGetPr(prhs[0]);
    double* inTransformData = mxGetPr(prhs[1]);

    int n_wires = inDimImage[1];
    int n_observations = inDimHomogenenous[2];

    int solution_size = 6+4*n_wires;
    mxArray* initial_guess = nullptr;
    if(nrhs == 4) //initial guess provided
    {
        mwSize supplied_initial_guess_dimensions = mxGetNumberOfDimensions(prhs[3]);
        if(supplied_initial_guess_dimensions!=2)
            mexErrMsgTxt("The initial guess must have a size of [1 6+4*n_wires] or [6+4*n_wires 1]");
        
        const mwSize * supplied_initial_guess_dim = mxGetDimensions(prhs[3]);

        bool dimensions_proper = ((supplied_initial_guess_dim[0] == 1 && supplied_initial_guess_dim[1] == solution_size) || 
           (supplied_initial_guess_dim[0] == solution_size && supplied_initial_guess_dim[1] == 1));
        if(!dimensions_proper)
            mexErrMsgTxt("The initial guess must have a size of [1 6+4*n_wires] or [6+4*n_wires 1]");

        initial_guess = mxDuplicateArray(prhs[3]);
    } else {
        initial_guess = mxCreateDoubleMatrix(solution_size,1,mxREAL);
    }

    plhs[0] = mxCreateDoubleMatrix(solution_size,1,mxREAL);
    double* initGuessData = mxGetPr(initial_guess);
    double* outSolutionData = mxGetPr(plhs[0]);

    // now we can call the solver once we have dealt with all of these complications
    cpp_solver_calibration(n_wires,n_iterations,n_observations,inImageData,inTransformData,initGuessData,outSolutionData);

    mxDestroyArray(initial_guess);

    return;
}