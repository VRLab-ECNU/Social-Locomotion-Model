clc;clear;
addpath('./functions');
load('./data/fit_parameters.mat');
m = x(1);
n = x(2);
a = 0.24;
b = 0.12;
c = x(3);
th = x(4)*10;
r = 5;
width = 10; % How wide is the map
EndPoint = [999;500];
Target = [0,10]; % The position of the target pole
precision = 0.01; % How precise is the map
baseSpeed = 1.36; % m/s
fps = 70; timestep = 1/fps;
minStep = baseSpeed/precision * timestep; % cm/frame
RTsettings = struct('useFD',1,'stepSize',1);
thinkx = (0:minStep:400)';
thinky = ones(length(thinkx),1)*500;
think = [thinky,thinkx];
pathlengthL = [];pathlengthR = [];
socialEnergyL = [];socialEnergyR = [];
TimeL = [];TimeR = [];
subpathall = [];

%% Generate path if MUST left
for ii = 1:5 % Avatar Start Place
    for j = 1:3 % Speed
        socialImpact = 0;
        deg = 30*ii - 90;
        speed = (0.2*j + 0.6) * baseSpeed;
        disp(['Processing LEFT Direction = ' num2str(deg) ', Speed = ' num2str(0.2*j + 0.6) '...']);
        r1 = r - speed*(1/baseSpeed);
        BlockerStart = [width/2 + r1*sind(deg),r + r1*cosd(deg)];
        Blocker_Ori = deg * -1;
        StartPoint = [100;500];
        
        timepoints = length(100:minStep:1000);
        subPath = zeros(timepoints,2);
        ini_avaPath = zeros(timepoints,2);
        avastep = [speed*timestep*sind(deg) , speed*timestep*cosd(deg)] .* -1/precision;
        for time = 1:timepoints
            subPath(time,:) = [500,100] + [0,minStep]*(time-1);
            ini_avaPath(time,:) = BlockerStart/precision + avastep*(time-1);
        end
        
        avaPath = ini_avaPath;
        num = 1;
        while 1
            d = subPath - avaPath;
            d1 = d(:,1).^2 + d(:,2).^2;
            LD = avaPath(:,2)-subPath(:,2);
            LD1 = abs(subPath(abs(LD)==min(abs(LD)),1) - avaPath(abs(LD)==min(abs(LD)),1));
%             disp(['Minimum distance along path: ' num2str(sqrt(min(d1)))]);
%             disp(['Minimum lateral distance along path: ' num2str(LD1)]);
%             disp(['Avatar Position: ' num2str(avaPath(d1==min(d1),2))]);
%             disp(['Subject Position: ' num2str(subPath(d1==min(d1),2))]);
            if num > 1
                break
            end
            Blocker = avaPath(d1==min(d1),:);
            EnergyMap = GetMap(width/precision+1,Target(2)/precision+1,Blocker,Blocker_Ori,m,n,a,b,c,precision);
            EnergyMap = rescale(EnergyMap,0,1,'InputMin',0,'InputMax',th);
            % Key manipulation %
            EnergyMap(round(Blocker(1)):(width/precision+1),round(Blocker(2))) = 1;
            % Key manipulation %
            SpeedMap = 1 - EnergyMap;
            T = fm(SpeedMap,StartPoint,[1 1],struct('implementation','C++','order',2));
            path = rayTrace(T,EndPoint,StartPoint,RTsettings);
            path=[path,flip(StartPoint)];
            l = arclength(path(1,:),path(2,:));
            subPath1 = interparc(round(l/minStep)+1,path(1,:),path(2,:));
            subPath1 = flip(subPath1);
            subPath = subPath1(1:length(avaPath),:);
            num = num + 1;
        end
        subPath1(:,2) = subPath1(:,2) + 300;
        subPath1 = flip([think;subPath1],2);
        subPath1(:,2) = (subPath1(:,2) - 500) * -1;

        for jj = 1:length(path)-1
            socialImpact = socialImpact + EnergyMap(round(path(1,j)),round(path(2,j)));
        end
        socialEnergyL = [socialEnergyL;socialImpact];
        subpathall = [subpathall;[ones(length(subPath1),1)*ii,ones(length(subPath1),1)*(0.2*j + 0.6),zeros(length(subPath1),1),subPath1]];
    end
end

%% Generate path if MUST right
for ii = 1:5 % Avatar Start Place
    for j = 1:3 % Speed
        socialImpact = 0;
        deg = 30*ii - 90;
        speed = (0.2*j + 0.6) * baseSpeed;
        disp(['Processing RIGHT Direction = ' num2str(deg) ', Speed = ' num2str(0.2*j + 0.6) '...']);
        r1 = r - speed*(1/baseSpeed);
        BlockerStart = [width/2 + r1*sind(deg),r + r1*cosd(deg)];
        Blocker_Ori = deg * -1;
        StartPoint = [100;500];
        
        timepoints = length(100:minStep:1000);
        subPath = zeros(timepoints,2);
        ini_avaPath = zeros(timepoints,2);
        avastep = [speed*timestep*sind(deg) , speed*timestep*cosd(deg)] .* -1/precision;
        for time = 1:timepoints
            subPath(time,:) = [500,100] + [0,minStep]*(time-1);
            ini_avaPath(time,:) = BlockerStart/precision + avastep*(time-1);
        end
        
        avaPath = ini_avaPath;
        num = 1;
        while 1
            d = subPath - avaPath;
            d1 = d(:,1).^2 + d(:,2).^2;
            LD = avaPath(:,2)-subPath(:,2);
            LD1 = abs(subPath(abs(LD)==min(abs(LD)),1) - avaPath(abs(LD)==min(abs(LD)),1));
%             disp(['Minimum distance along path: ' num2str(sqrt(min(d1)))]);
%             disp(['Minimum lateral distance along path: ' num2str(LD1)]);
%             disp(['Avatar Position: ' num2str(avaPath(d1==min(d1),2))]);
%             disp(['Subject Position: ' num2str(subPath(d1==min(d1),2))]);
            if num > 1
                break
            end
            Blocker = avaPath(d1==min(d1),:);
            EnergyMap = GetMap(width/precision+1,Target(2)/precision+1,Blocker,Blocker_Ori,m,n,a,b,c,precision);
            EnergyMap = rescale(EnergyMap,0,1,'InputMin',0,'InputMax',th);
            % Key manipulation %
            EnergyMap(1:round(Blocker(1)),round(Blocker(2))) = 1;
            % Key manipulation %
            SpeedMap = 1 - EnergyMap;
            T = fm(SpeedMap,StartPoint,[1 1],struct('implementation','C++','order',2));
            path = rayTrace(T,EndPoint,StartPoint,RTsettings);
            path=[path,flip(StartPoint)];
            l = arclength(path(1,:),path(2,:));
            subPath1 = interparc(round(l/minStep)+1,path(1,:),path(2,:));
            subPath1 = flip(subPath1);
            subPath = subPath1(1:length(avaPath),:);
            num = num + 1;
        end
        subPath1(:,2) = subPath1(:,2) + 300;
        subPath1 = flip([think;subPath1],2);
        subPath1(:,2) = (subPath1(:,2) - 500) * -1;
        for jj = 1:length(path)-1
            socialImpact = socialImpact + EnergyMap(round(path(1,j)),round(path(2,j)));
        end
        socialEnergyR = [socialEnergyR;socialImpact];
        subpathall = [subpathall;[ones(length(subPath1),1)*ii,ones(length(subPath1),1)*(0.2*j + 0.6),ones(length(subPath1),1),subPath1]];
    end
end

socialEnergyRatio = socialEnergyL./socialEnergyR;

subpathall = subpathall(subpathall(:,4)<1270,:);
subpathall = [subpathall,zeros(length(subpathall),1)];
% dlmwrite('./results/path_dynamic.csv',subpathall);
% plot(path(1,:),path(2,:))
x = (1:1270)';
newsubpathall = [];
for ii=0:1
    for j=1:5
        for k=0.8:0.2:1.2
            temp = subpathall(subpathall(:,3)==ii & subpathall(:,1)==j & subpathall(:,2)==k,:);
            y = interp1(temp(:,4),temp(:,5),x,'linear','extrap');
            newsubpathall = [newsubpathall;[ones(1270,1)*j,ones(1270,1)*k,ones(1270,1)*ii,x,y,zeros(1270,1)]];
        end
    end
end
newsubpathall = sortrows(newsubpathall,[1 2 3]);
dlmwrite('./results/path_dynamic(social).csv',newsubpathall);
save('./results/dynamicpath(social).mat','newsubpathall')

realdata = dlmread('./data/final.csv');
realP = realdata(:,3);
pathlengthRatio = pathlengthL(:,3)./pathlengthR(:,3);
socialEnergyRatio = socialEnergyL(:,3)./socialEnergyR(:,3);
% 1./(1+exp(a+b*x))
modelPrediction = [realP,socialEnergyRatio];
dlmwrite('./results/corD_s.csv',modelPrediction);