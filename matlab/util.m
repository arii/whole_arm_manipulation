classdef util
    methods(Static)
        
        function plot_line_segment(a, b, color)
            if nargin > 2
                plot([a(1) b(1)], [a(2) b(2)], color, 'linewidth', 3)
            else
                plot([a(1) b(1)], [a(2) b(2)], 'linewidth', 3)
            end
        end
        
        function plot_pt(pt, markersize)
            if nargin < 2
                s = 10
            else
                s = markersize
            end
            
            plot (pt(1), pt(2), '*', 'markersize', s)
        end
        
        function plotPretty(filename)
            % Plot some stuff
            % Making figure tight
            ti = get(gca, 'TightInset');
            set(gca, 'Position',[ti(1) ti(2) 1-ti(3)-ti(1) 1-ti(4)-ti(2)]);
            set(gca, 'units', 'centimeters');
            pos = get(gca,'Position');
            ti = get(gca,'TightInset');
            set(gcf, 'PaperUnits', 'centimeters');
            set(gcf, 'PaperSize', [pos(3)+ti(1)+ti(3) pos(4)+ti(2)+ti(4)]);
            set(gcf, 'PaperPositionMode', 'manual');
            set(gcf, 'PaperPosition',...
                [0 0 pos(3)+ti(1)+ti(3) pos(4)+ti(2)+ti(4)]);
            if nargin > 0
                % Save figure to PDF
                saveas(gcf, filename);
            end
        end

  
        function u = unit_vector(v)
            u = v/norm(v,2);
        end
            


        
    end
end
