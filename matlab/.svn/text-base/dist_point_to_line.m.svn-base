function dist_point_to_line(p, p0, p1)
 v = p1-p0
 w = p - p0
 c1 = dot(w,v)
 c2 = dot(v,v)
 b = c1/c2
 pb = p0 + b*v
 
 
 q0 = pb - p0
 q1 = pb - p1
 if norm(q0,2) > norm(q1,2)
     util.plot_line_segment(p1,pb, 'r')
 else
     util.plot_line_segment(p0,pb, 'r')
 end
  hold on
 util.plot_line_segment(pb,p, 'g')
 util.plot_pt(pb)
end
