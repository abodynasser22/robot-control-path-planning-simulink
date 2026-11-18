classdef LivePlot3D < matlab.System
    % Live 3D plot of UAV position

    properties(Access = private)
        hFig
        hLine
    end

    methods(Access = protected)
        function setupImpl(obj, x, y, z)
            % Called once at start of simulation
            obj.hFig  = figure('Name','UAV 3D trajectory');
            obj.hLine = plot3(x, y, z, '.');
            grid on; axis equal;
            xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');
            hold on;
        end

        function stepImpl(obj, x, y, z)
            % Called each sample time
            if ~isvalid(obj.hFig)
                return; % fönstret stängt
            end
            xd = [get(obj.hLine,'XData') x];
            yd = [get(obj.hLine,'YData') y];
            zd = [get(obj.hLine,'ZData') z];
            set(obj.hLine,'XData',xd,'YData',yd,'ZData',zd);
            drawnow limitrate;  % uppdatera utan att bli för tungt
        end
    end
end
