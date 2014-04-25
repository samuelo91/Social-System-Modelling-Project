function [x,y] = testModel(steps)

%Global variables
NOAGENTS = 30;
START_DISTANCE = 0.2
dt = 0.1

%agent = [x, y, vx, vy, desVel, ex, ey, vavg]
%agents = [agent1; agent2; agent3;...]
agents = zeros(NOAGENTS,8);
agentsUpdated = zeros(NOAGENTS,8);

%Initalize agents at the left side with y distance START_DISTANCE apart
for a = 1:NOAGENTS
    agent = [0,a*START_DISTANCE,0,0,1.3,1,0,0];
    agents(a,:) = agent;
end

for time = 1:dt:steps
    
    for a = 1:NOAGENTS   
        agent = agents(a,:);
        
        %Acceleration force of every agent is calculated
        [accFx,accFy] = accelerationF(agent(6), agent(7), agent(3), agent(4), agent(8));
        
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
    set (gca, 'YLimMode', 'Manual', 'YLim', [-5 35], 'XLim', [0 30]);
    drawnow
    %0.1 seconds pause so the agents move in 'realtime'
    pause(0.1);
end

agents
end



%Average speed of the agent, considering current step in time
function avgSpeed = averageSpeed(old_avg, newSpeed, time)
avgSpeed = (old_avg * (time-1) + newSpeed) / time;
end


% nervousness, inital desired velocity is set to 1
function nerv = nervousness(avgSpeed)
nerv= 1-avgSpeed/1;
end

% acceleration force from own desired direction
function [accFx, accFy] = accelerationF(desX, desY, vx, vy, vavg)
n = nervousness(vavg);
desVel = (1-n)*1 + n*1.6;
accFx = desVel*desX-vx;
accFy = desVel*desY-vy;
end

%force from one other pedestrian
function [pedFx, pedFy] = pedestrianF(x, y, otherx, othery)

d = [x,y]-[otherx,othery];
pedFx = 3*exp(0.6-norm(d)/0.2)* d(1)/norm(d);
pedFy = 3*exp(0.6-norm(d)/0.2)* d(2)/norm(d);

end



