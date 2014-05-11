function testModel(steps,filename, situation)
%filename has to be a string (e.g. 'test.mat')

%Global variables
NOAGENTS = 10;
SPEED_MEAN = 1.3;
SPEED_DISTR = 0.1;
dt = 0.05;
framesNo = (steps-1)/dt+1;

%agent = [x, y, vx, vy, desVel, type, unused, vavg, waypointNO]
%agents = [agent1; agent2; agent3;...]
agents = zeros(NOAGENTS,9);
agentsUpdated = zeros(NOAGENTS,9);
positionDataX = zeros(NOAGENTS,framesNo);
positionDataY = zeros(NOAGENTS,framesNo);
forceData = zeros(15,15,framesNo);
pplSqData = zeros(15,15,framesNo);

if(situation == 0)
    %Init destination zone
    waypointsA = [10,6,10,9;
                  15,5,15,10;
                  25,5,25,10];
    waypointsB = [6,5,9,5;
                  5,0,10,0;
                  5,-10,10,-10];

    %walls form a cross
    walls = [-10,5,5,5;
             -10,10,5,10;
             10,5,15,5;
             10,10,15,10;
             5,0,5,5;
             5,10,5,25;
             10,0,10,5;
             10,10,10,25];
         
    %walls = [walls; makeObstacleRect(7,7,8,8)];
    %walls = [walls; makeObstacleTriangle(7,7,8,8,6,9)];      
end

if(situation == 1)
    %Init destination zone
    waypointsA = [0,7,7,7;
                  7,7,2,14;
                  7,7,7,14;
                  15,7,15,15;
                  25,7,25,15];
    waypointsB = [7,7,7,14;
                  7,7,2,14;
                  0,7,7,7;
                  0,0,7,0;
                  0,-10,7,-10];

    %walls form a curve
    walls = [0,-10,0,15;
             0,15,25,15;
             7,-10,7,7;
             7,7,25,7];
end

%Initalize agents at the left side with y distance START_DISTANCE apart
for a = 1:NOAGENTS
    % Sets the desired speed of the agent in a normaldistribution with
    % SPEED_MEAN +- SPEED_DISRTR
    speed = SPEED_MEAN + sqrt(SPEED_DISTR)*randn;
    
    if(situation == 0)
        % random number between a and b: a+(b-a)*rand
        if(a<NOAGENTS/2)
            posx = -5 + (2-(-2))*rand;
            posy = 5 + (10-5)*rand;
            agent = [posx,posy,0,0,speed,0,0,speed,1];
        else
            posx = 5 + (10-5)*rand;
            posy = -5 + (2-(-2))*rand;
            agent = [posx,15-posy,0,0,speed,1,-1,speed,1];
        end
    end
    
    if(situation == 1)
        % random number between a and b: a+(b-a)*rand
        if(a<NOAGENTS/2)
            posx = 0 + (7-(0))*rand;
            posy = -5 + (0-(-5))*rand;
            agent = [posx,posy,0,0,speed,0,0,speed,1];
        else
            posx = 15 + (20-15)*rand;
            posy = 0 + (7-(0))*rand;
            agent = [posx,15-posy,0,0,speed,1,-1,speed,1];
        end
    end
    
    agents(a,:) = agent;
end

counter=1;
% time loop
for time = 1:dt:steps
    pplSqUnit = zeros(15,15);
    % agent loop
    for a = 1:NOAGENTS   
        agent = agents(a,:);
        
        %Set up desired destination for each agent
        if(agent(6)==0)
            [ex,ey,d] = vectorFromWall(waypointsA(agent(9),:),agent(1),agent(2));
        else
            [ex,ey,d] = vectorFromWall(waypointsB(agent(9),:),agent(1),agent(2));
        end
        if(d<1)
            [s,dontcare] = size(waypointsA);
            if (agent(9) < s)
                agent(9) = agent(9)+1;
                if(agent(6)==0)
                    [ex,ey,d] = vectorFromWall(waypointsA(agent(9),:),agent(1),agent(2));
                else
                    [ex,ey,d] = vectorFromWall(waypointsB(agent(9),:),agent(1),agent(2));
                end
            end
        end    
        
        %Acceleration force of every agent is calculated
        [accFx,accFy] = accelerationF(-ex, -ey, agent(3), agent(4), agent(8), agent(5)); 
        
        %Wall forces for every agent
        mind = 10000;
        [noWalls,dontcare] = size(walls);
        for w = 1:noWalls
            wall = walls(w,:);
            [vx,vy,d] = vectorFromWall(wall,agent(1),agent(2)); 
            if (d<mind)
                mind = d;
                minn = [vx,vy];
            end
        end
        
        [wallFx, wallFy] = wallF(mind, minn);
        agent(3) = agent(3) + dt*wallFx;
        agent(4) = agent(4) + dt*wallFy; 
        
        %Simplified Force from other agents (only considered second part of
        %formula)
        
        for i = 1:NOAGENTS
            
            if (a~=i)
                %Do calc for all other agents except oneself
                otheragent = agents(i,:);
                [pedFx,pedFy] = pedestrianF(agent(1),agent(2),otheragent(1),otheragent(2));
                %Add pedestrian force to velocity
                agent(3) = agent(3) + dt*pedFx;
                agent(4) = agent(4) + dt*pedFy;
            end
              
        end
        %Add acceleration force to velocity
        agent(3) = agent(3) + dt*accFx;
        agent(4) = agent(4) + dt*accFy;
        %Update position according to new velocity
        agent(1) = agent(1) + dt*agent(3);
        agent(2) = agent(2) + dt*agent(4);
        if(ceil(agent(2)) >0 & ceil(agent(1)) >0 & ceil(agent(2))<=15 & ceil(agent(1))<=15);
            pplSqUnit(ceil(agent(2)),ceil(agent(1))) = pplSqUnit(ceil(agent(2)),ceil(agent(1)))+1;
        end
        %Update average speed
        agent(8) = averageSpeed(agent(8),sqrt(agent(3)^2 + agent(4)^2), time);
        
        %We use a new matrix agentsUpdated so all agents are updated in one
        %step and for updates depending on the others positions the old
        %positions of already updated agents are used
        agentsUpdated(a,:) = agent;
        
    end
    
    agents = agentsUpdated;
    positionDataX(:,counter) = agents(:,1);
    positionDataY(:,counter) = agents(:,2);
    pplSqData (:,:,counter) = pplSqUnit;
    counter = counter+1;

%     plot(agents(:,1),agents(:,2),'Marker', 'o','LineStyle', 'none','MarkerSize', 10)
%     set (gca, 'YLimMode', 'Manual', 'YLim', [0 15], 'XLim', [0 15]);
%     [m,n] = size(walls);
%     for w = 1 : m
%         line([walls(w,1);walls(w,3)],[walls(w,2);walls(w,4)])
%     end
%     drawnow
%     %dt seconds pause so the agents move in 'realtime'
%     pause(dt);
end

% Plot positions from saved data matrices
% for time = 1:framesNo
%     hold on
%     imagesc(0:15,0:15,pplSqData(:,:,time),[0,8]); 
%     colormap jet;
%     colorbar
%     plot(positionDataX(:,time),positionDataY(:,time), 'Marker', 'o','LineStyle', 'none','MarkerSize', 10, 'MarkerEdgeColor','k')
%     set (gca, 'YLimMode', 'Manual', 'YLim', [0 15], 'XLim', [0 15]);
%     [m,n] = size(walls);
%     for w = 1 : m
%         line([walls(w,1);walls(w,3)],[walls(w,2);walls(w,4)],'Color','k','LineWidth',2)
%     end
%     %drawnow
%     %dt seconds pause so the agents move in 'realtime'
%     pause(dt);
%     hold off
% end
save(filename);

end

%Average speed of the agent, considering current step in time
function avgSpeed = averageSpeed(old_avg, newSpeed, time)
avgSpeed = (old_avg * (time-1) + newSpeed) / time;
end

% nervousness, inital desired velocity is given by desSpeed
function nerv = nervousness(avgSpeed, desSpeed)
nerv= 1-avgSpeed/desSpeed;
end

% acceleration force from own desired direction
function [accFx, accFy] = accelerationF(desX, desY, vx, vy, vavg, desSpeed)
n = nervousness(vavg, desSpeed);
desVel = (1-n)*desSpeed + n*(desSpeed*1.3);
accFx = desVel*desX-vx;
accFy = desVel*desY-vy;
end

%force from walls
function [wallFx, wallFy] = wallF(mind, n)

wallFx = 5*exp((0.3-mind)/0.1) * n(1);
wallFy = 5*exp((0.3-mind)/0.1) * n(2);

end


%force from one other pedestrian
function [pedFx, pedFy] = pedestrianF(x, y, otherx, othery)

d = [x,y]-[otherx,othery];
pedFx = 3*exp((0.6-norm(d))/0.2)* d(1)/norm(d);
pedFy = 3*exp((0.6-norm(d))/0.2)* d(2)/norm(d);

end

function [vx,vy,d] = vectorFromWall(wall,x,y)

    vectorA = [wall(1),wall(2)] - [wall(3),wall(4)];
    vectorB = [x,y] - [wall(3),wall(4)];
    l2 = norm(vectorA)^2;
    t = dot(vectorB,vectorA)/l2;
    if(t<0)
        n = vectorB;
        d = norm(n);
    else if(t>1)
        n = [x,y] - [wall(1),wall(2)];
        d = norm(n);
        else
            n = [-vectorA(2),vectorA(1)];
            d = abs(det([vectorA;vectorB])) / norm(vectorA);
            n = n/norm(n)*sign(dot(n,vectorB));
        end
    end
    vx = n(1);
    vy = n(2);
    
end

function obstacle = makeObstacleRect(x,y,x2,y2)

obstacle = [x,y,x,y2;
            x,y2,x2,y2;
            x2,y2,x2,y;
            x2,y,x,y];
        
end

function obstacle = makeObstacleTriangle(x,y,x2,y2,x3,y3)

obstacle = [x,y,x2,y2;
            x2,y2,x3,y3;
            x3,y3,x,y];
        
end





