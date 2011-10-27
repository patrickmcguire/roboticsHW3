function [ returnVal ] = bug2a( serPort )
%bug2 Implementation of Bug2 Algorithm
%Carmine Elvezio, Chris M. Hauteur, Patrick McGuire
%   Details
    % wall-follow Carmine
    %   queries ?on M Line??
    %   breaks out on true
    % get current Position
    %   return x, y, theta (relative to origin and 0 degrees)
    % on M Line (given position and tolerance, returns boolean) Pattycake
    %   queries getCurrentPosition
    %   deals with hitpoint position relative to other hitpoints
    % at X (given position and tolerance, returns boolean) Chris
    %   Goal?
    %   last hit point
    % is Closer? (global lastHP, getCurrentPos > returns boolean)
    % remember furthest hit point
    % We are turning LEFT
    % overall algorithm
    %   define goal pt. as 10 meters in front of robot
    %   define tolerance as .5 meters or whatever

    initialVal = [0,0];
    current = initialVal;
    finalVal = [2,0];
    tolerance = [0.1, 0.1];
    lastHit = [0,0];
    newHeading = 0;

    maxVelocity = 0.2;
    maxIncreasVel = 0.005;
    maxOdometerAngle = pi/4;
    maxTime = 1200;
    maxDistanceSinceBump = 5;
    once = false;
        
    v = maxVelocity;
    w = (maxVelocity-v)/0.2;
    startTime = tic;

    keepMoving = true;
    bumped = false;
    returnState = 0;
    reachedEnd = false;
    
    wallVal = 0;
    
    figure(2);
    clf; % clear figure 1
    hold on; 

    DistanceSensorRoomba(serPort);
    AngleSensorRoomba(serPort);
    
    turnAndSetup(serPort, finalVal, current);%careful
    
    SetFwdVelAngVelCreate(serPort,v,w);
    
    while(keepMoving)
        
        %SetFwdVelAngVelCreate(serPort,v,w);
        figure(2);
        plot(current(1), current(2), 'r:+');

        if(atX(current(1), current(2),  finalVal(1), finalVal(2)))
            keepMoving = false;
            reachedEnd = true;
            SetFwdVelAngVelCreate(serPort,0,0);
        end

        [bumped current newHeading] = reactBump(serPort, current, newHeading);
        if (bumped)
            lastHit = current;
            %reactBump(serPort);
            [returnState current newHeading] = wallFollow(serPort, current, initialVal, finalVal, lastHit, newHeading);
            
            if(returnState == 0)
                %reached end
                'Stopped through Wall Follow'
                keepMoving = false;
                reachedEnd = true;
            elseif(returnState == 1)
                %CANNOT reach end
                keepMoving = false;
                reachedEnd = false;
            elseif(returnState == 2)
                %keep moving, set new q to leave point
                keepMoving = true;
                %turnAndSetup(serPort, finalVal, current);%careful
            end
        end
        
        pause(0.1)
    end
    
   %SetFwdVelAngVelCreate(serPort,0,0);
   'Stopped Moving!'
   if(reachedEnd)
      'Reached end'
   else
      'End cannot be reached'
   end
   
end

function [bumpedInto current newHeading] = reactBump(serPort, current, newHeading)

    [current(1) current(2) newHeading] = getCurrentPosition(serPort, current(1), current(2), newHeading);
    
    [BumpRight BumpLeft WheDropRight WheDropLeft WheDropCaster ... 
        BumpFront] = BumpsWheelDropsSensorsRoomba(serPort);

    bumpedInto = BumpRight || BumpLeft || BumpFront;

    if bumpedInto
        figure(2);
        plot(current(1),current(2)+0.01, 'd');
        
        ang = 0;
        
        wallVal = WallSensorReadRoomba(serPort);
        if BumpRight && wallVal == 1
            ang = pi/8;
        elseif BumpLeft
            ang = pi/4;
        else
            ang = pi/2;
        end
        SetFwdVelAngVelCreate(serPort,0,0);
        turnAngle(serPort, 0.2, (ang*180)/pi);
        
    end
end

function turnAndSetup(serPort, final, current)
    ang = atan2(final(2), final(1))
    turnAngle(serPort,0.2,(ang*180)/pi);

end



