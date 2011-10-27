function [coincident] = atX(x1,y1,x2,y2)
    TOLERANCE = 0.1; % meters
    
    %[x2 y2 ~] = getCurrentPosition();
    
    dx = abs(x1-x2);
    dy = abs(y1-y2);
    
    if (dx <= TOLERANCE && dy <= TOLERANCE) 
        coincident = true;
    else
        coincident = false;
    end
end



% function [closer] = isCloser()
% % Inputs:
% %   lastHitPoint
% %   current position
% %
% % Outputs:
% %   1  if robot is closer to the goal
% %   0  if robot is farther from the goal
% %   -1 if robot is at the same place (within tolerance)
% 
%     ROBOT_CLOSER = 1;
%     ROBOT_FARTHER = 0;
%     ROBOT_TRAPPED = -1;
% 
%     [lastX lastY] = lastHitPoint;
%     [goalX goalY] = GOAL;
%     [currX currY ~] = getCurrentPosition();
%     
%     lastdx = abs(goalX - lastX);
%     lastdy = abs(goalY - lastY);
%     lastd = sqrt( lastdx^2 + lastdy^2 );
%     
%     currdx = abs(goalX - currX);
%     currdy = abs(goalY - currY);
%     currd = sqrt( currdx^2 + currdy^2 );
%     
%     if (currd < lastd )
%         closer = ROBOT_CLOSER;
%     elseif (currd > lastd)
%         closer = ROBOT_FARTHER;
%     elseif (atX(lastX,lastY,currX,currY) == true)
%         closer = ROBOT_TRAPPED;
%     end
% end
% 
% function [onMLine] = onMLine()
% 
%     % Distance between current position and the m-line
%     [ax ay] = START;
%     [bx by] = GOAL;
%     [px py ~] = getCurrentPosition();
%     
%     ab = [ bx - ax, by - ay ];                      % vector ab
%     abnorm = norm(ab);                              % unit ab vector
%     ab = [ ab(1)/abnorm, ab(2)/abnorm ];            % normalize ab
%     n = [ -ab(2), ab(1) ];                          % normal to ab unit vector
%     ap = [ px - ax, py - ay ];                      % vector ap
%     minDist = abs( ap(1) * n(1) + ap(2) * n(2) );   % projection of ap onto n 
%     
%     if (minDist < TOLERANCE)
%         onMLine = true;
%     else
%         onMLine = false;
%     end
% end
