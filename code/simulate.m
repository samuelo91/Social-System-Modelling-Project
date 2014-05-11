function simulate(filename,mode)
%filename has to be a string (e.g. 'test.mat')

load(filename);
h = figure();

vidObj = VideoWriter('video.avi');
open(vidObj);

for time = 1:framesNo
    if(mode==1)
        hold on
        imagesc(0:15,0:15,pplSqData(:,:,time),[0,8]); 
        colormap jet;
        colorbar
    end
    plot(positionDataX(:,time),positionDataY(:,time), 'Marker', 'o','LineStyle', 'none','MarkerSize', 10, 'MarkerEdgeColor','k')
    set (gca, 'YLimMode', 'Manual', 'YLim', [0 15], 'XLim', [0 15]);
    [m,n] = size(walls);
    for w = 1 : m
        line([walls(w,1);walls(w,3)],[walls(w,2);walls(w,4)],'Color','k','LineWidth',2)
    end
    %drawnow
    %dt seconds pause so the agents move in 'realtime'
    pause(dt);
    if(mode==1)
        hold off
    end
    
    currFrame = getframe(h);
    writeVideo(vidObj,currFrame);
    
end

close(vidObj);

end

