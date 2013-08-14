function d =  compute_dist(p, p0, p1)
    v = p1-p0
    w = p - p0
    c1 = dot(w,v)
    c2 = dot(v,v)
    
    clf
    hold on
    
    if c1 <= 0 
       
    else
        if c2 <= c1
            d = 2
        else
        d = 3
        b = c1/c2
        pb = p0 + b*v
        util.plot_pt(pb)
        util.plot_line_segment(p,pb)
        end
    end

    util.plot_line_segment(p,p0)
    util.plot_line_segment(p0,p1)
    
    
    
end
