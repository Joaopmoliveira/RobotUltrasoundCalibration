function R = eul2rot(euler_vector)
%EUL2ROT Summary of this function goes here
%   Detailed explanation goes here

t1 = cos(euler_vector(3));
t2 = sin(euler_vector(3));
t3 = cos(euler_vector(2));
t4 = sin(euler_vector(2));
t5 = cos(euler_vector(1));
t6 = sin(euler_vector(1));

R = [ t1*t3, t1*t4*t6 - t2*t5, t2*t6 + t1*t4*t5 ;
      t2*t3, t1*t5 + t2*t4*t6, t2*t4*t5 - t1*t6;
        -t4,            t3*t6,            t3*t5];

end

