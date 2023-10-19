addpath('matlab_utilities');
addpath('bin');

num_lines = 3;
noise_level = 0;

%% In this file we follow the next nomenclature
%   frame_0 = base frame
%   frame_1 = flange frame
%   frame_2 = image frame
%   point_3 = image intersection with line
%   R01 - rotation of frame 1 in relation to frame 0
%   p01 -translation of frame 1 in relation to frame 0

%% Transformation of image to flange
theta_2 = [0.2 0.2 0.2];
R12 = eul2rot(theta_2);

% position of the image frame in the flange frame;
p12 = [0.1 0.2 0.15]';

subplotToShow = figure('units','normalized','outerposition',[0.25 0.25 0.5 0.5]);

%% Base coordinate frame
plot3([0 0.2],[0 0],[0 0],'b','LineWidth',3)
hold on;
plot3([0 0],[0 0.2],[0 0],'b','LineWidth',3)
plot3([0 0],[0 0],[0 0.2],'b','LineWidth',3)

title('3D representation of automatic line and transform estimation');

%% Phantom frame definition
% In this code we define a phatom which has several fields, namely the
% Transform which contains rotation and translational information about the
% position of the phatom in the base frame, the lenght of the phantom as
% d_x, d_y, and d_z distances and lastly the information about the lines
% contained in the phaton in its coordinates

%phatom.R=eul2rot([deg2rad(13) deg2rad(10) deg2rad(5)]);
phatom.R = eul2rot([0 0 10]);
phatom.p=[0.1 0.1 0.1]';
phatom.d_x = 0.05; %5 cm
phatom.d_y = 0.2; %20 cm
phatom.d_z = 0.05; %5 cm

% In the problem simulation we define the equation by the Hesse convention as
% such: sin(theta)*cos(phi)*x +sin(theta)*sin(phi)*y + cos(theta)*z -d = 0
% to simplify the use by the developer the actual line is defined by two
% points in the phantom frame in the plane Oxz and the parallel
% Oxz+phantom_length_y plane


switch(num_lines)

    case 1 % point

        %%%%%%%%%%%%%%%%  <1 line> %%%%%%%%%%%%%%%%%%%%%%%%
        p1Oxz = [0.025 0.0 0.025];
        p1OxzOffset = [0.025 phatom.d_y 0.025];
        %%%%%%%%%%%%%%%% </1 line> %%%%%%%%%%%%%%%%%%%%%%%%
        phatom.unprocessed_data=[ p1Oxz p1OxzOffset];

    case 2 % line
        %%%%%%%%%%%%%%%%  <1 line> %%%%%%%%%%%%%%%%%%%%%%%%
        p1Oxz = [0.0 0.0 0.0];
        p1OxzOffset = [0.0 phatom.d_y 0.0];
        %%%%%%%%%%%%%%%% </1 line> %%%%%%%%%%%%%%%%%%%%%%%%
        %%%%%%%%%%%%%%%%  <1 line> %%%%%%%%%%%%%%%%%%%%%%%%
        p2Oxz = [0.05 0.0 0.05];
        p2OxzOffset = [0.05 phatom.d_y 0.05];
        %%%%%%%%%%%%%%%% </1 line> %%%%%%%%%%%%%%%%%%%%%%%%
        phatom.unprocessed_data=[ p1Oxz p1OxzOffset ; p2Oxz p2OxzOffset];

    case 3 % triangle
        %%%%%%%%%%%%%%%%  <1 line> %%%%%%%%%%%%%%%%%%%%%%%%
        p1Oxz = [0.0 0.0 0.0];
        p1OxzOffset = [0.0 phatom.d_y 0.0];
        %%%%%%%%%%%%%%%% </1 line> %%%%%%%%%%%%%%%%%%%%%%%%
        %%%%%%%%%%%%%%%%  <1 line> %%%%%%%%%%%%%%%%%%%%%%%%
        p2Oxz = [0.05 0.0 0.05];
        p2OxzOffset = [0.05 phatom.d_y 0.05];
        %%%%%%%%%%%%%%%%  <1 line> %%%%%%%%%%%%%%%%%%%%%%%%
        p3Oxz = [0.0 0.0 0.05];
        p3OxzOffset = [0.0 phatom.d_y 0.05];
        %%%%%%%%%%%%%%%% </1 line> %%%%%%%%%%%%%%%%%%%%%%%%
        phatom.unprocessed_data=[ p1Oxz p1OxzOffset ; p2Oxz p2OxzOffset ; p3Oxz p3OxzOffset ];

    case 4 % square
        %%%%%%%%%%%%%%%%  <1 line> %%%%%%%%%%%%%%%%%%%%%%%%
        p1Oxz = [0.0 0.0 0.0];
        p1OxzOffset = [0.0 phatom.d_y 0.0];
        %%%%%%%%%%%%%%%% </1 line> %%%%%%%%%%%%%%%%%%%%%%%%
        %%%%%%%%%%%%%%%%  <1 line> %%%%%%%%%%%%%%%%%%%%%%%%
        p2Oxz = [0.05 0.0 0.05];
        p2OxzOffset = [0.05 phatom.d_y 0.05];
        %%%%%%%%%%%%%%%%  <1 line> %%%%%%%%%%%%%%%%%%%%%%%%
        p3Oxz = [0.0 0.0 0.05];
        p3OxzOffset = [0.0 phatom.d_y 0.05];
        %%%%%%%%%%%%%%%% </1 line> %%%%%%%%%%%%%%%%%%%%%%%%
        p4Oxz = [0.05 0.0 0.0];
        p4OxzOffset = [0.05 phatom.d_y 0.0];
        %%%%%%%%%%%%%%%% </1 line> %%%%%%%%%%%%%%%%%%%%%%%%
        phatom.unprocessed_data=[ p1Oxz p1OxzOffset ; p2Oxz p2OxzOffset ; p3Oxz p3OxzOffset ; p4Oxz p4OxzOffset ];

    otherwise
        error('cannot compute values with more than four sides\n');

end

plotPhantom(phatom);
%set(gca, 'color', [0.1 0.1 0.1]);
phatom.lines = checkLineData(phatom);

solution.sol = [theta_2 p12'];
solution = processLinesForOptimalSolution(solution,phatom);

angle_y_range = 30;


%% Trajectory of the flange the reconstruct the line of the phantom

% number of data points to generate
trajectory.points = 50;
% angle around the x axis
%trajectory.angle_x = deg2rad([-90 -85 -70 -60 -80 -90 -100 -120 -100 -90 -70]);
trajectory.angle_x = deg2rad([ -100 -90 -70 -90 -100]);

% angle around the y axis
%trajectory.angle_y = deg2rad([0 7 0 -7 0]);
trajectory.angle_y = deg2rad([0 30 0 -30 0]);

% angle around the z axis
%trajectory.angle_z = deg2rad([0 10 20 35 50 20 5 -10 -20 -35 -45 -30 -14 5 0]);
trajectory.angle_z = deg2rad([ 0 10 0 -10 0]);

trajectory.x_motion=[0 0.1 0];
trajectory.y_motion=[0 1 0 1];
trajectory.z_motion = [0 0 0];
% the solution that we follow to paremeterize a trajectory depending only
% on a single variable, (time for example) in relation to the phatom frame.
% Since we know all the transformations involved we can compute the
% position and rotation of the flange in the base frame

% Generate the distance from the image to the line and the corresponding R01 and p01
data = define_flange_trajectory(p12,R12,phatom,trajectory);

% Generate the distance from the image to the line and the corresponding R01 and p01
T12 = zeros(4,4);
T12(1:3,1:3) = R12;
T12(1:3,4) = p12;
T12(4,4)=1;
data = compute_multiple_intersections(data,T12,phatom);

data = addNoiseToImageData(data,noise_level);
indices = squeeze(sum(sum(data.W(:,:,:),1),2))~=inf;
ImageData = data.W(:,:,indices);
TransformData = data.T(1:3,:,indices);

estimated_solution=CalibrationSolver(ImageData,TransformData);

fprintf("The true solution of the problem is: \n");
for i = 1 : length(solution.sol)
    fprintf(" %f ",solution.sol(i));
end

fprintf("\n");

fprintf("The estimated solution of the problem is: \n");
for i = 1 : length(estimated_solution)
    fprintf(" %f ",estimated_solution(i));
end

fprintf("\n");