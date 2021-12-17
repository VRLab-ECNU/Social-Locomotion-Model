clc;clear;

%% Read Condition
load('../data/condition_demo.mat');
mapWidth = condition{1};
Start = condition{2};
Target = condition{3};
Blocker = condition{4};
Blocker_Ori = condition{5};

%% Generate the Path
path = GeneratePath(mapWidth,Start,Target,Blocker,Blocker_Ori);
save('../output/path_demo.mat','path');

%% Plot the Path
figure;
l = plot(path(:,2),path(:,1),'k','LineWidth',3);
text(100*Start(2)+10,100*Start(1)+25,'Start Point','FontSize',16);
text(100*Target(2)-110,100*Target(1)+25,'Target Point','FontSize',16);
xlim([0,500]);ylim([400,600]);
xlabel('X (cm)');ylabel('Y (cm)');
daspect([1 1 1]);
set(gca,'FontSize',16);
saveas(gcf,'../output/path_demo.png');