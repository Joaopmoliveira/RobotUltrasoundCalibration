function solution = processLinesForOptimalSolution(solution,phantom)

lines = phantom.lines;

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