function[P] = GetPoint(style)
            button = 0; isZoom = 0;
            while (button ~= 27 && button ~= 1)
                if (isZoom == 0)
                    title('Left-click to select a point | Right-click to zoom in')
                    xlim('auto'); % zoom out
                    ylim('auto'); % zoom out
                else
                    title('Left-click to select a point');
                    xlim([P(1)-300, P(1)+300]); % zoom in
                    ylim([P(2)-300, P(2)+300]); % zoom in                    
                end
                [P(1), P(2), button] = ginput(1);
                if (button == 1)              
                    plot(P(1), P(2), style);
                end
                if (button == 3) % if the right button is clicked
                   isZoom = 1 - isZoom;
                end
            end
            xlim('auto'); % zoom out
            ylim('auto'); % zoom out
        end