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

%% Noise corruption from different sources

data = addNoiseToImageData(data,noise_level);
% % % % % %
% % % % % % %% Solve nonlinear optimization problem
[ImageData,TransformData] = compute_dual_plane_cost_function_vector(data);

%[ep12,eR12,esolution,jacobian]=solve_nonlinear_cost_levenberg_marquardt(cost,solution.sol+rand(1,length(solution.sol))*10^(-3));

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

function line_data = checkLineData(phatom)
temporary_data=phatom.unprocessed_data;
line_data=zeros(size(temporary_data));
for i = 1 : size(temporary_data,1)
    if(temporary_data(i,2)~=0||temporary_data(i,5)~=phatom.d_y)
        error('The two points that define the line in the phantom do not belong to the plane Oxz or the Oxz+length_y');
    end
    if((~(temporary_data(i,1)>=0.0&&temporary_data(i,1)<=phatom.d_x))||(~(temporary_data(i,3)>=0.0&&temporary_data(i,3)<=phatom.d_z)))
        error('A point from one of the lines is not contained in the area of the phantom in the Oxz plane');
    end
    if((~(temporary_data(i,4)>=0.0&&temporary_data(i,4)<=phatom.d_x))||(~(temporary_data(i,6)>=0.0&&temporary_data(i,6)<=phatom.d_z)))
        error('A point from one of the lines is not contained in the area of the phantom in the Oxz plane+ length_y');
    end
    vec_line=temporary_data(i,4:6)-temporary_data(i,1:3);
    line_data(i,:)=[((phatom.R*vec_line')*(1/norm(vec_line)))' (phatom.p+phatom.R*temporary_data(i,1:3)')'];
    p1=phatom.p+phatom.R*temporary_data(i,1:3)';
    p2=phatom.p+phatom.R*temporary_data(i,4:6)';
    plot3([p1(1) p2(1)],[p1(2) p2(2)],[p1(3) p2(3)],'r','LineWidth',3);
end
end

%%%-------------------------------------------------------------------------------------------------------%%%
function plotPhantom(phatom)

p0 = phatom.p;
p1 = phatom.p+phatom.R*[phatom.d_x 0 0]';
p2 = phatom.p+phatom.R*[phatom.d_x 0 phatom.d_z]';
p3 = phatom.p+phatom.R*[0 0 phatom.d_z]';
p4 = phatom.p+phatom.R*[0 phatom.d_y phatom.d_z]';
p5 = phatom.p+phatom.R*[0 phatom.d_y 0]';
p6 = phatom.p+phatom.R*[phatom.d_x phatom.d_y 0]';

face1 = fill3([p0(1) p1(1) p2(1) p3(1)],[p0(2) p1(2) p2(2) p3(2)],[p0(3) p1(3) p2(3) p3(3)],'w');
alpha(face1,1);
hold on;
axis equal;
grid on;
face2 = fill3([p0(1) p3(1) p4(1) p5(1)],[p0(2) p3(2) p4(2) p5(2)],[p0(3) p3(3) p4(3) p5(3)],'w');
alpha(face2,1);
face3 = fill3([p0(1) p1(1) p6(1) p5(1)],[p0(2) p1(2) p6(2) p5(2)],[p0(3) p1(3) p6(3) p5(3)],'w');
alpha(face3,1);

end


function [ImageData,TransformData] = compute_dual_plane_cost_function_vector(data)
indices = squeeze(sum(sum(data.W(:,:,:),1),2))~=inf;

ImageData = data.W(:,:,indices);
TransformData = data.T(1:3,:,indices);
end

%%%--------------------------------------------------------------------------------------------------------%%%
function data=define_flange_trajectory(p12,R12,phantom,trajectory)

%data = repmat(struct('p01',[0;0;0],'R01',eye(3),'b23',zeros(3,size(phantom.lines,1))),trajectory.points,1);
data.T = zeros(4,4,trajectory.points);
data.W = zeros(3,size(phantom.lines,1),trajectory.points);

T_flange_to_image = [R12' -R12'*p12; 0 0 0 1];
T_phantom_to_base = [phantom.R phantom.p ; 0 0 0 1];

distance_x = spline(0:1/(length(trajectory.x_motion)-1):1 , trajectory.x_motion , 0:1/(trajectory.points-1):1)*phantom.d_x;
distance_y =  spline(0:1/(length(trajectory.y_motion)-1):1 , trajectory.y_motion , 0:1/(trajectory.points-1):1)*phantom.d_y;
distance_z =  spline(0:1/(length(trajectory.z_motion)-1):1 , trajectory.z_motion , 0:1/(trajectory.points-1):1)*phantom.d_z;

angle_x = spline(0:1/(length(trajectory.angle_x)-1):1 , trajectory.angle_x , 0:1/(trajectory.points-1):1);
angle_y = spline(0:1/(length(trajectory.angle_y)-1):1 , trajectory.angle_y , 0:1/(trajectory.points-1):1);
angle_z = spline(0:1/(length(trajectory.angle_z)-1):1 , trajectory.angle_z , 0:1/(trajectory.points-1):1);

for i = 1 : trajectory.points
    R_image_to_phantom = eul2rot( [angle_x(i) , angle_y(i) , angle_z(i)] );
    p_image_to_phantom = [distance_x(i) distance_y(i) distance_z(i)]';
    T_image_to_phantom = [R_image_to_phantom p_image_to_phantom; 0 0 0 1];
    data.T(:,:,i) = T_phantom_to_base*T_image_to_phantom*T_flange_to_image;
end

end


%%%--------------------------------------------------------------------------------------------------------%%%
function data = compute_multiple_intersections(data,T12,phatom)
number_of_lines =size(phatom.lines,1);
for i = 1 : size(data.T,3)
    for j = 1 : number_of_lines
        data.W(:,j,i)= computeIntersection(data.T(:,:,i),T12,phatom.lines(j,:));
    end
end

% now we check if the position of the points in space lies inside the
% phantom, if not we remove the point

for i = 1 : size(data.T,3)
    for j = 1 : number_of_lines
        p03 = data.T(:,:,i)*T12*[data.W(:,j,i); 1];
        pdiff = phatom.R'*(p03(1:3)-phatom.p);
        if(pdiff(1)-eps>phatom.d_x || pdiff(1)+eps<0)       
            data.W(:,j,i) = realmax('double');
        elseif(pdiff(2)-eps>phatom.d_y || pdiff(2)+eps<0)
            data.W(:,j,i) = realmax('double');
        elseif(pdiff(3)-eps>phatom.d_z || pdiff(3)+eps<0)
            data.W(:,j,i) = realmax('double');
        else
            plot3(p03(1),p03(2),p03(3),'w*');
        end
    end
end
end

%%%--------------------------------------------------------------------------------------------------------%%%
function p23 = computeIntersection(T01,T12,line_data)

T02 = T01*T12;

perpVec_to_imag_plane =T02(1:3,3);
parallel_line_vec = line_data(1:3)';
point_of_line = line_data(4:6)';
plane_d = T02(1:3,4)'*perpVec_to_imag_plane;

p0xy = T02*[0 0 0 1]';
p0xpyp = T02*[0.1 0 0 1]';
p0xyp = T02*[0.1 0.1 0 1]';
p0xpy = T02*[0 0.1 0 1]';

face3 = fill3([p0xy(1) p0xpyp(1) p0xyp(1) p0xpy(1)],[p0xy(2) p0xpyp(2) p0xyp(2) p0xpy(2)],[p0xy(3) p0xpyp(3) p0xyp(3) p0xpy(3)],'b');
alpha(face3,0.1);

K= (1/(parallel_line_vec'*perpVec_to_imag_plane))*(plane_d-point_of_line'*perpVec_to_imag_plane);
p03 = point_of_line+K*parallel_line_vec ;
p23 = T12(1:3,1:3)'*(T01(1:3,1:3)'*(p03-T01(1:3,4))-T12(1:3,4));
end


%%%--------------------------------------------------------------------------------------------------------%%%
function plotPointsInSpace(data,T12,colorchar,vec)
for i = 1 : size(data.T,3)
    [~,ind]=find(data.W(:,:,i) == realmax);
    Trans =  data.W(:,:,i);
    Trans(:,ind)=[];
    % compute intersection in base frame
    if (~isempty(Trans))
        p03 = data.T(:,:,i)*T12*[Trans; ones(1,size(Trans,2))];
        scatter3(p03(1,:),p03(2,:),p03(3,:),colorchar,'MarkerFaceColor',vec);
        hold on;
    end
end
end


%%%--------------------------------------------------------------------------------------------------------%%%
function data = addNoiseToImageData(data,noise_ratio)

data.W =data.W+ noise_ratio*2*(rand(3,size(data.W,2),size(data.W,3))-repmat(0.5,[3 size(data.W,2) size(data.W,3)]));

end


%%% -------------------------------------------------------------------------------------------------------%%%
function solution = processLinesForOptimalSolution(solution,phantom)

lines = phantom.lines;

[X,Y] = meshgrid(phantom.p(1):phantom.d_x/3:phantom.p(1)+phantom.d_x,phantom.p(2):phantom.d_y/3:phantom.p(2)+phantom.d_y);

current_solution = solution.sol;
new_solution = zeros(1,4*size(lines,1));
index = 1;
for i = 1 : size(lines,1)
    perpendicular_vec = cross(lines(i,1:3)',lines(i,4:6)');
    perpendicular_vec = perpendicular_vec*(1/norm(perpendicular_vec));
    normal_to_virtual_plane = cross(perpendicular_vec',lines(i,1:3)');
    phi = atan2(normal_to_virtual_plane(2),normal_to_virtual_plane(1));
    if(phi>-eps && phi< eps)    
        theta = atan2(-normal_to_virtual_plane(3),normal_to_virtual_plane(2));
    else
        theta = atan2(-normal_to_virtual_plane(3),normal_to_virtual_plane(2)/sin(phi));
    end
    alpha = atan2(-perpendicular_vec(3),lines(i,3));
    d = -lines(i,4:6)*normal_to_virtual_plane';
    new_solution(index:index+3) = [ alpha , theta , phi , d ];
  
    index=index+4;
end
solution.sol = [current_solution , new_solution];
end