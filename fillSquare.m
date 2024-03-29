mapRadius = 20;
origin = [mapRadius, mapRadius];
map = zeros(2*mapRadius,2*mapRadius);
map(:) = 0.5;
createRadius = 0.6; % XXX This is probably wrong!

% isOccupied is either 1 (occupied) or 0 (not occupied)
% x and y are in meters, relative to the origin
function fillSquare(x,y,isOccupied)    

    sx = ceil(x/createRadius);
    sy = ceil(y/createRadius);
    map( origin(1) + sx, origin(2) + sy ) = not(isOccupied);
    colormap(gray(3))
    imagesc(map)
end

%outline = [];
function setOutlinePoint(x,y)

end

function renderOutline()

end
