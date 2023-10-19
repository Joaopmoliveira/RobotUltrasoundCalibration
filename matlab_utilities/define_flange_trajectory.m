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