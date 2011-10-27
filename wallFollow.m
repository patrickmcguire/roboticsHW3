function [ returnVal current newHeading ] = wallFollow( serPort, current, initialVal, finalVal, lastHit, newHeading )
%bug2 Implementation of Bug2 Algorithm- wall follower
%Carmine Elvezio, Chris M. Hauteur, Patrick McGuire
%Add parameters

    %Max Variables
    maxVelocity = 0.2;
    maxIncreasVel = 0.005;
    maxOdometerAngle = pi/4;
    maxTime = 1200;
    incidentTime = 5;
    maxDistanceSinceBump = 5;
    
    %velocity + angular
    v = maxVelocity;
    w = (maxVelocity-v)/0.2;
    startTime = tic;
    hitTime = tic;

    completeSpin = false;
    once = true;
    
    ROBOT_CLOSER = 1;
    ROBOT_FARTHER = 0;
    ROBOT_TRAPPED = -1;
    desired_theta = atan2(finalVal(2), finalVal(1))

    SetFwdVelAngVelCreate(serPort, v,w);
    
    
    %Stop if run too long & stop if we have looped
    while toc(startTime) < maxTime && completeSpin == false;
        
        wallVal = WallSensorReadRoomba(serPort);
        if wallVal
            figure(2);
            plot (current(1)+0.01, current(2), '*');
        end

        if wallVal == 0 && once == true
            v = maxVelocity * 0.45;%.33 Optimal
            w=-1*(maxVelocity-v)/0.2;
            SetFwdVelAngVelCreate(serPort, v,w);
        elseif wallVal == 1 && once == true
            v = maxVelocity * 0.9;
            w=1*(maxVelocity-v)/0.2;
            SetFwdVelAngVelCreate(serPort, v,w);
        elseif wallVal == 0 && once == false %specific straight case
            pause(0.1);
            v = maxVelocity;
            w=1*(maxVelocity-v)/0.2;
            SetFwdVelAngVelCreate(serPort, v,w);            
        end
        
        
        %Do Bump Computation
        [BumpRight BumpLeft WheDropRight WheDropLeft WheDropCaster ... 
            BumpFront] = BumpsWheelDropsSensorsRoomba(serPort);
        bumped = BumpRight || BumpLeft || BumpFront;
        
        if bumped
            figure(2);
            plot(current(1), current(2)+0.01, 'd');
            %AngleSensorRoomba(serPort);%watch
            ang = 0;
            if BumpRight
                ang = pi/8;
            elseif BumpLeft
                ang = pi/4;
            else
                ang = pi/2;
            end
            SetFwdVelAngVelCreate(serPort,0,0);
            turnAngle(serPort, 0.2, (ang*180)/pi);
            SetFwdVelAngVelCreate(serPort, v,w);
        end
        
        %
        % Get position
        [current(1) current(2) newHeading] = getCurrentPosition(serPort, current(1), current(2), newHeading);

        
        [onML angTurnM] = onMLine2(initialVal, finalVal, current);
            
        if onML & toc(hitTime) > 10
            'inML'
            
            if(atX(current(1), current(2),  finalVal(1), finalVal(2)))
                returnVal = 0;
                SetFwdVelAngVelCreate(serPort,0,0);
                break;
            end
            if(isCloser(current, finalVal, lastHit) == ROBOT_CLOSER)
                SetFwdVelAngVelCreate(serPort,0,0);
                
                'Desired Theta'
                (desired_theta*180)/pi
                'New Heading'
                (newHeading*180)/pi
                'Ang Turn P'
                angTurnP = desired_theta - newHeading
                'In Radians'
                (angTurnP*180)/pi
                
                turnAngle(serPort, 0.2, ((angTurnP*180)/pi)-1);
                SetFwdVelAngVelCreate(serPort,maxVelocity,0);
                returnVal = 2;
                break;
                
            elseif(isCloser(current, finalVal, lastHit) == ROBOT_TRAPPED)
                SetFwdVelAngVelCreate(serPort,0,0);
                returnVal = 1;
                break;
            
            else
                SetFwdVelAngVelCreate(serPort,maxVelocity,0);
                
            end
        end
        
        figure(2);
        plot(current(1), current(2), 'r:+');
        
        %pause(0.1)
        
    end

end

function [closer] = isCloser(current, finalVal, lastHit)
% Inputs:
%   lastHitPoint
%   current position
%
% Outputs:
%   1  if robot is closer to the goal
%   0  if robot is farther from the goal
%   -1 if robot is at the same place (within tolerance)

    ROBOT_CLOSER = 1;
    ROBOT_FARTHER = 0;
    ROBOT_TRAPPED = -1;

    %[lastX lastY] = lastHit;
    lastX = lastHit(1);
    lastY = lastHit(2);
    %[goalX goalY] = finalVal;
    goalX = finalVal(1);
    goalY = finalVal(2);
    %[currX currY] = current;
    currX = current(1);
    currY = current(2);
    
    lastdx = abs(goalX - lastX);
    lastdy = abs(goalY - lastY);
    lastd = sqrt( lastdx^2 + lastdy^2 );
    
    currdx = abs(goalX - currX);
    currdy = abs(goalY - currY);
    currd = sqrt( currdx^2 + currdy^2 );
    
    if (currd < lastd )
        closer = ROBOT_CLOSER;
    elseif (currd > lastd)
        closer = ROBOT_FARTHER;
    elseif (atX(lastX,lastY,currX,currY) == true)
        closer = ROBOT_TRAPPED;
    end
end

function [onMLine] = onMLine(initialVal, finalVal, current)

    % Distance between current position and the m-line

    ax = initialVal(1);
    ay = initialVal(2);
    
    bx = finalVal(1);
    by = finalVal(2);
    
    px = current(1);
    py = current(2);
    TOLERANCE = 0.1;
    
    ab = [ bx - ax, by - ay ];                      % vector ab
    abnorm = norm(ab);                              % unit ab vector
    ab = [ ab(1)/abnorm, ab(2)/abnorm ];            % normalize ab
    n = [ -ab(2), ab(1) ];                          % normal to ab unit vector
    ap = [ px - ax, py - ay ];                      % vector ap
    minDist = abs( ap(1) * n(1) + ap(2) * n(2) );   % projection of ap onto n 
    
    if (minDist < TOLERANCE)
        onMLine = true
    else
        onMLine = false;
    end
end


function [OnMLine ang] = onMLine2(initialVal, finalVal, current)
desired_theta = atan2(finalVal(2), finalVal(1))
current_theta = atan2(current(2), current(1))
ang = current_theta - desired_theta;
    if abs(current_theta - desired_theta) < .03;
        OnMLine = true;
        return;
    else
        OnMLine = false;
        return;
    end
end