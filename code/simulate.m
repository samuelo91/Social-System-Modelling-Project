function simulate(filename,mode,savevideo)
%filename has to be a string (e.g. 'test.mat')
%mode is either 0 (no background color with people per square) or 1
%savevideo has to be a string (e.g. 'video.avi')

load(filename);
h = figure();

vidObj = VideoWriter(savevideo);
vidObj.FrameRate = 20;
open(vidObj);

for time = 1:framesNo
    %plot with people per square or without
    if(mode==1)
        hold on
        %plot people per square
        imagesc(0:15,0:15,pplSqData(:,:,time),[0,8]);
        colormap jet;
        cb = colorbar('vert');
        zlab = get(cb,'ylabel');
        set(zlab,'String','people per square meter','fontsize',14); 
    end
    %plot agents in black
    plot(positionDataX(:,time),positionDataY(:,time), 'Marker', 'o','LineStyle', 'none','MarkerSize', 10, 'MarkerEdgeColor','k')
    tlhand = get(gca,'title');
    set(tlhand,'string',sprintf('Agents in xy space over time'),'fontsize',16);
    xlhand = get(gca,'xlabel');
    set(xlhand,'string','x-position','fontsize',14);
    ylhand = get(gca,'ylabel');
    set(ylhand,'string','y-position','fontsize',14);
    set (gca, 'YLimMode', 'Manual', 'YLim', [0 15], 'XLim', [0 15]);
    [m,n] = size(walls);
    for w = 1 : m
        line([walls(w,1);walls(w,3)],[walls(w,2);walls(w,4)],'Color','k','LineWidth',2)
    end
    if(mode==1)
        hold off
    end
    
    currFrame = getframe(h);
    writeVideo(vidObj,currFrame);
    
end

close(vidObj);

end

