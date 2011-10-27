% Global data
mapRadius = 6;
origin = [mapRadius, mapRadius];  % robot starts at center of map
map = zeros(2*mapRadius,2*mapRadius);
map(:) = 0.5;   % fill map with grey

% isOccupied is either 1 (occupied) or 0 (not occupied)
% x and y are in meters, relative to the origin
function fillSquare(gX,gY,isOccupied)    
    map( origin(1) + gX, origin(2) + gY ) = not(isOccupied);

end

function [gX,gY] = currentGridCoords(x,y)
    createRadius = 0.6; % XXX This is probably wrong!
    gX = ceil(x/createRadius);
    gY = ceil(y/createRadius);
end

function drawMap()
    colormap(gray(3))
    imagesc(map)
end

% function setOutlinePoint(x,y)
% end

% function renderOutline()
% end