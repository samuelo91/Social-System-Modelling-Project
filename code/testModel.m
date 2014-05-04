function [x,y] = testModel(steps)

%Global variables
NOAGENTS = 60;
START_DISTANCE = 0.3;
SPEED_MEAN = 1.1;
SPEED_DISTR = 0.1;
dt = 0.05

%agent = [x, y, vx, vy, desVel, ex, ey, vavg]
%agents = [agent1; agent2; agent3;...]
agents = zeros(NOAGENTS,8);
agentsUpdated = zeros(NOAGENTS,8);

%Initalize Walls
walls = zeros(8,4);
%TODO add walls
%walls = (0,5,5,5;0,10,5,10;10,5,15,5;10,10,15,10;5,0,5,5;5,10,5,15);

%Initalize agents at the left side with y distance START_DISTANCE apart
for a = 1:NOAGENTS
    % Sets the desired speed of the agent in a normaldistribution with
    % SPEED_MEAN +- SPEED_DISRTR
    speed = SPEED_MEAN + sqrt(SPEED_DISTR)*randn;
    % random number between a and b: a+(b-a)*rand
    posx = -2 + (2+2)*rand;
    %posx=0;
    if(a<NOAGENTS/2)
        agent = [posx,a*START_DISTANCE,0,0,speed,1,0,speed];
    else
        agent = [15-posx,(a-NOAGENTS/2)*START_DISTANCE,0,0,speed,-1,0,speed];
    end
    
    agents(a,:) = agent;
end

% time loop
for time = 1:dt:steps
    
    % agent loop
    for a = 1:NOAGENTS   
        agent = agents(a,:);
        
        %Acceleration force of every agent is calculated
        [accFx,accFy] = accelerationF(agent(6), agent(7), agent(3), agent(4), agent(8), agent(5));
        
        %Wall forces for every agent
        
        
        %Simplified Force from other agents (only considered second part of
        %formula)
        
        for i = 1:NOAGENTS
            
            if (a~=i)
                %Do calc for all other agents except oneself
                otheragent = agents(i,:);
                [pedFx,pedFy] = pedestrianF(agent(1),agent(2),otheragent(1),otheragent(2));
                agent(3) = agent(3) + dt*pedFx;
                agent(4) = agent(4) + dt*pedFy;
            end
              
        end
        
        agent(3) = agent(3) + dt*accFx;
        agent(4) = agent(4) + dt*accFy;
        agent(1) = agent(1) + dt*agent(3);
        agent(2) = agent(2) + dt*agent(4);
        agent(8) = averageSpeed(agent(8),sqrt(agent(3)^2 + agent(4)^2), time);
        
        %We use a new matrix agentsUpdated so all agents are updated in one
        %step and for updates depending on the others positions the old
        %positions of already updated agents are used
        agentsUpdated(a,:) = agent;
        
    end
    
    agents = agentsUpdated;
    
    plot(agents(:,1),agents(:,2),'Marker', 'o','LineStyle', 'none')
    set (gca, 'YLimMode', 'Manual', 'YLim', [-5 10], 'XLim', [0 15]);
    drawnow
    %dt seconds pause so the agents move in 'realtime'
    pause(dt);
end

agents
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

function [wallFx, wallFy] = wallF(x, y, vx, vywalls)

end


%force from one other pedestrian
function [pedFx, pedFy] = pedestrianF(x, y, otherx, othery)

d = [x,y]-[otherx,othery];
pedFx = 3*exp((0.6-norm(d))/0.2)* d(1)/norm(d);
pedFy = 3*exp((0.6-norm(d))/0.2)* d(2)/norm(d);

end



