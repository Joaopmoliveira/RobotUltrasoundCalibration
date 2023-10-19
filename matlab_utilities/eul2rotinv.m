function [euler_vector] = eul2rotinv(R)

z = atan2(R(2,1),R(1,1));

y = atan2(-R(3,1),sqrt(R(3,2)^2+R(3,3)^2));

x = atan2(R(3,2),R(3,3));

euler_vector = [x,y,z];

end

