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