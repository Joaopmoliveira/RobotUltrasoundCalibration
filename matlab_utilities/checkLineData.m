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