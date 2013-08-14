function plot_perp_dist(p, p0, p1, a1)
clf
a0 =p1;
x = linspace(0, 2*pi);
compute_dist(p, p0, p1)
compute_dist(p, a0, a1)
plot(.5*cos(x), 0.5*sin(x)+1, 'linewidth', 3)
dist_point_to_line(p, p0, p1)
dist_point_to_line(p, a0, a1)
end
