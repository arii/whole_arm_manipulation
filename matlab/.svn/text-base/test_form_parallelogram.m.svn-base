
function test_form_parallelogram(end_points)
    % 3 points right now
    v1 = end_points(2,:) - end_points(1,:);
    v2 = end_points(3,:) - end_points(2,:);
    
    p1 = end_points(1,:)
    p2 = end_points(2,:)
    p3 = end_points(3,:)
    p4 = p1 + v2
    clf
    hold on
    plot_line_segment(p1,p2)
    plot_line_segment(p2,p3)
    plot_line_segment(p3, p4)
    plot_line_segment(p4,p1)
    
    function plot_line_segment(a, b)
        plot([a(1) b(1)], [a(2) b(2)])
    end

    point_in_convex_polygon([p1;p2;p3;p4])
    

end
