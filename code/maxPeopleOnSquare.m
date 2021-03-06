function maxPeopleOnSquare(inputfile)

load(inputfile);
ppsVsTime = zeros(framesNo);
maxPerSquare = zeros(15,15);

for time = 1:framesNo
    ppsVsTime(time) = max(max(pplSqData(:,:,time)));
end

for i = 1:15
    for j = 1:15
        maxPerSquare(i,j) = max(pplSqData(i,j,:));
    end
end
hFig = figure;
set(gcf,'PaperPositionMode','auto')
set(hFig, 'Position', [0 0 1070 405])
subplot(1,2,1)
plot(ppsVsTime)
axis([0 framesNo 0 8])
tlhand = get(gca,'title');
set(tlhand,'string',sprintf('number of people \n on the square meter with highest density \n at each frame'),'fontsize',20);
xlhand = get(gca,'xlabel');
set(xlhand,'string','frame','fontsize',18);
ylhand = get(gca,'ylabel');
set(ylhand,'string','max people/m^2','fontsize',18);
subplot(1,2,2)
hold on
imagesc(0:15,0:15,maxPerSquare,[0,8]);
[m,n] = size(walls);
for w = 1 : m
    line([walls(w,1);walls(w,3)],[walls(w,2);walls(w,4)],'Color','k','LineWidth',2)
end
axis([0 15 0 15])
tlhand = get(gca,'title');
set(tlhand,'string',sprintf('maximal number of people \n on each square meter \n in xy-space'),'fontsize',20);
colormap jet;
cb = colorbar('vert');
zlab = get(cb,'ylabel');
set(zlab,'String','people per square meter','FontSize',18);
xlhand = get(gca,'xlabel');
set(xlhand,'string','x-position','fontsize',18);
ylhand = get(gca,'ylabel');
set(ylhand,'string','y-position','fontsize',18);
%print -depsc2 test.eps; 
hold off