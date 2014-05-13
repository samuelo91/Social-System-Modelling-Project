function testModel(steps,filename, situation, noAgents)
%filename has to be a string (e.g. 'test.mat')
%situation is either 0 --> cross situation
%                    1 --> curve situation

%save the seed on the Random Stream Generator so the results can be
%reproduced
stream = RandStream.getGlobalStream;

%Global variables
NOAGENTS = noAgents;
SPEED_MEAN = 1.3;
SPEED_DISTR = 0.1;
dt = 0.05;
framesNo = (steps-1)/dt+1;

%agent = [x-position, y-position, x-speed, y-speed, desired Velocity, type (which start side), waypointNO, average speed]
%agents = [agent1; agent2; agent3;...]
agents = zeros(NOAGENTS,8);
agentsUpdated = zeros(NOAGENTS,8);
positionDataX = zeros(NOAGENTS,framesNo);
positionDataY = zeros(NOAGENTS,framesNo);
pplSqData = zeros(15,15,framesNo);

if(situation == 0)
    %Init waypoints to reach destination
    waypointsA = [10,6,10,9;
                  15,5,15,10;
                  16,5,16,10];
    waypointsB = [6,5,9,5;
                  5,0,10,0;
                  5,-1,10,-1];

    %walls form a cross
    walls = [-20,5,5,5;
             -20,10,5,10;
             10,5,35,5;
             10,10,35,10;
             5,-10,5,5;
             5,10,5,35;
             10,-10,10,5;
             10,10,10,35];
         
    %Obstacles are added here:
    walls = [walls; makeObstacleRect(7,7,8,8)];
    %walls = [walls; makeObstacleTriangle(7,7,8,8,6,9)];
    %walls = [walls;7,4,8,5;8,6,9,7;10,7,11,8];

end

if(situation == 1)
    %Init waypoints to reach destination
    waypointsA = [2,7,7,7;
                  3,13,8,8;
                  15,7,15,15;
                  16,7,16,15];
    waypointsB = [7,7,7,13;
                  1,11,6,6;
                  0,0,7,0;
                  0,-1,7,-1];

    %walls form a curve
    walls = [0,-10,0,15;
             0,15,25,15;
             7,-10,7,7;
             7,7,25,7];
         
    %Obstacles are added here: 
    %walls = [walls;5,7,7,9;4,8,6,10;3,9,5,11];
    %walls = [walls;4.5,7.5,6.5,9.5;3,9,5,11];
end

%Initalize agents according to situation
for a = 1:NOAGENTS
    
    % Sets the desired speed of the agent in a normaldistribution with
    % SPEED_MEAN +- SPEED_DISRTR
    speed = SPEED_MEAN + sqrt(SPEED_DISTR)*randn;
    
    if(situation == 0)
        %agents on left side
        if(a<=NOAGENTS/2)
            posx = -5 + 4*rand;
            posy = 5 + (10-5)*rand;
            agent = [posx,posy,speed,0,speed,0,1,speed];
        %agents on top    
        else
            posx = 5 + (10-5)*rand;
            posy = -5 + 4*rand;
            agent = [posx,15-posy,0,-speed,speed,1,1,speed];
        end
    end
    
    if(situation == 1)
        %agents at the bottom
        if(a<=NOAGENTS/2)
            posx = 0 + (7-(0))*rand;
            posy = -5 + (0-(-5))*rand;
            agent = [posx,posy,0,speed,speed,0,1,speed];
        %agents at the right side    
        else
            posx = 15 + (20-15)*rand;
            posy = 0 + (8-(0))*rand;
            agent = [posx,15-posy,-speed,0,speed,1,1,speed];
        end
    end
    
    agents(a,:) = agent;
end

counter=1;
%----------------START OF SIMULATION---------------------------------------
% time loop
for time = 1:dt:steps
    pplSqUnit = zeros(15,15);
    %-----------FOR EACH AGENT DO CALCULATIONS-----------------------------
    % agent loop
    for a = 1:NOAGENTS   
        agent = agents(a,:);
        
        %------Set up desired destination for the agent-------------------
        if(agent(6)==0)
            [ex,ey,d] = vectorFromWall(waypointsA(agent(7),:),agent(1),agent(2));
        else
            [ex,ey,d] = vectorFromWall(waypointsB(agent(7),:),agent(1),agent(2));
        end
        % waypoint is reached
        if(d<0.5)
            [s,dontcare] = size(waypointsA);
            %agent not at en-destination yet -> set next waypoint
            if (agent(7) < s)
                agent(7) = agent(7)+1;
            %agent at end-destination -> set agent back into start area
            else
                speed = SPEED_MEAN + sqrt(SPEED_DISTR)*randn;

                if(situation == 0)
                    if(agent(6)==0)
                        posx = -5 + (2-(-2))*rand;
                        posy = 5 + (10-5)*rand;
                        agent = [posx,posy,speed,0,speed,0,1,speed];
                    else
                        posx = 5 + (10-5)*rand;
                        posy = -5 + (2-(-2))*rand;
                        agent = [posx,15-posy,0,-speed,speed,1,1,speed];
                    end
                end

                if(situation == 1)
                    if(agent(6)==0)
                        posx = 0 + (7-(0))*rand;
                        posy = -5 + (0-(-5))*rand;
                        agent = [posx,posy,0,speed,speed,0,1,speed];
                    else
                        posx = 15 + (20-15)*rand;
                        posy = 0 + (8-(0))*rand;
                        agent = [posx,15-posy,-speed,0,speed,1,1,speed];
                    end
                end
            end
            if(agent(6)==0)
                [ex,ey,d] = vectorFromWall(waypointsA(agent(7),:),agent(1),agent(2));
            else
                [ex,ey,d] = vectorFromWall(waypointsB(agent(7),:),agent(1),agent(2));
            end
        end    
        
        %-------Acceleration force of every agent is calculated------------
        [accFx,accFy] = accelerationF(-ex, -ey, agent(3), agent(4), agent(8), agent(5)); 
        %Add acceleration force to velocity
        agent(3) = agent(3) + dt*accFx;
        agent(4) = agent(4) + dt*accFy;
        
        %-------Wall forces for every agent--------------------------------
        mind = 10000;
        [noWalls,dontcare] = size(walls);
        %calculate nearest wall
        for w = 1:noWalls
            wall = walls(w,:);
            [vx,vy,d] = vectorFromWall(wall,agent(1),agent(2)); 
            if (d<mind)
                mind = d;
                minn = [vx,vy];
            end
        end
        %Influence of nearest wall on agent
        [wallFx, wallFy] = wallF(mind, minn);
        agent(3) = agent(3) + dt*wallFx;
        agent(4) = agent(4) + dt*wallFy; 
        
        %------Force from other agents-------------------------------------
        
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

        %-------Update position according to new velocity------------------
        agent(1) = agent(1) + dt*agent(3);
        agent(2) = agent(2) + dt*agent(4);
        
        %------Save in which square meter the agent is in------------------
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
    %save data into matrices for plotting later
    positionDataX(:,counter) = agents(:,1);
    positionDataY(:,counter) = agents(:,2);
    pplSqData (:,:,counter) = pplSqUnit;
    counter = counter+1;

end

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

%the most direct vector from a wall to a position(x,y) and its length
function [vx,vy,d] = vectorFromWall(wall,x,y)

    vectorA = [wall(1),wall(2)] - [wall(3),wall(4)];
    vectorB = [x,y] - [wall(3),wall(4)];
    l2 = norm(vectorA)^2;
    t = dot(vectorB,vectorA)/l2;
    if(t<0)
        n = vectorB;
        d = norm(n);
        n = n/norm(n);
    else if(t>1)
        n = [x,y] - [wall(1),wall(2)];
        d = norm(n);
        n = n/norm(n);
        else
            n = [-vectorA(2),vectorA(1)];
            d = abs(det([vectorA;vectorB])) / norm(vectorA);
            n = n/norm(n)*sign(dot(n,vectorB));
        end
    end
    vx = n(1);
    vy = n(2);
    
end

%creates a rectangular obstacle out of walls
function obstacle = makeObstacleRect(x,y,x2,y2)

obstacle = [x,y,x,y2;
            x,y2,x2,y2;
            x2,y2,x2,y;
            x2,y,x,y];
        
end

%creates a triangular obstacle out of walls
function obstacle = makeObstacleTriangle(x,y,x2,y2,x3,y3)

obstacle = [x,y,x2,y2;
            x2,y2,x3,y3;
            x3,y3,x,y];
        
end





