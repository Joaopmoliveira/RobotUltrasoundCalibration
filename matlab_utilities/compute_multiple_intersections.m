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
