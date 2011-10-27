function [ x y newHeading ] = getCurrentPosition( serPort, x, y, newHeading )
%bug2 Implementation of Bug2 Algorithm - GetCurrentPosition includes x, y,
%and theta
%Carmine Elvezio, Chris M. Hauteur, Patrick McGuire
    
    %Do odometer computation
    distanceSegment = DistanceSensorRoomba(serPort);
    angleTurned = AngleSensorRoomba(serPort);
    newHeading = newHeading+angleTurned;

    deltaX = cos(newHeading)*distanceSegment;
    deltaY = sin(newHeading)*distanceSegment;


    x = x + deltaX;
    y = y + deltaY;

end

