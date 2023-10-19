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