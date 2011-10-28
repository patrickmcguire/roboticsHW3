classdef grid
    properties
        mapRadius;
        origin;
        map;
        createRadius;
    end
    % isOccupied is either 1 (occupied) or 0 (not occupied)
    % x and y are in meters, relative to the origin
    methods
        function gridObj = grid()
            gridObj.mapRadius = 20;
            gridObj.origin = [gridObj.mapRadius, gridObj.mapRadius];
            gridObj.map = zeros(2*gridObj.mapRadius,2*gridObj.mapRadius);
            gridObj.createRadius = 0.6; % XXX This is probably wrong!
         % Method help here
        end
        
        function [] = draw(gridObj)
            colormap(gray(3))
            imagesc(gridObj.map)
        end
        
        function [gridX, gridY] = gridCoordinates(gridObj,x,y)
            gridX = ceil(x/gridObj.createRadius) + gridObj.mapRadius/2;
            gridY = ceil(y/gridObj.createRadius) + gridObj.mapRadius/2;
        end
        
        function [v] = visited(gridObj,gridX,gridY)
            v = gridObj.map(gridX,gridY);
        end
        
        function [gridObj fresh] = setVisited(gridObj,gridX,gridY)
            if ~visited(gridObj,gridX, gridY)
                fresh = true;
            end
            gridObj.map(gridX,gridY) = 1;
        end
        
        function [gridObj fresh] = setCollision(gridObj,gridX,gridY)
            if ~visited(gridObj,gridX, gridY)
                fresh = true;
            end
            gridObj.map(gridX,gridY) = 2;
        end
    end
end

%outline = [];
%someday we will have a functino to figure the colors