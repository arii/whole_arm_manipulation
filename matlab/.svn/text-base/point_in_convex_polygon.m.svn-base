function inside = point_in_convex_polygon(vertices)
clf
hold on

p1 = vertices(length(vertices),:);
p2 = vertices (1,:);
a = compute_angle(p1,p2);
inside = true;

for i = 1 : length(vertices)-1
    p1 = vertices(i,:);
    p2 = vertices(i+1,:);
    tmp = compute_angle(p1,p2);
    inside = inside && (a == tmp);
   
end
p1 = vertices(length(vertices),:);
p2 = vertices (1,:);

compute_angle(p1,p2);
plot(0,0, '.', 'markersize', 20)
axis([-10 10 -10 10])
end


function angle = compute_angle(p1,p2)
    x1 = p1(1);
    x2 = p2(1);
    y1 = p1(2);
    y2 = p2(2);
    
    plot( [x1 x2], [y1 y2])
    
    angle = x2*y1 - x1*y2
    
    angle = (angle > 0);
    
end