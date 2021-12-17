clc;clear;
addpath('./functions');
load('./data/fit_parameters.mat');
m = x(1);
n = x(2);
a = 0.24;
b = 0.12;
c = x(3);
th = x(4)*10;
width = 10; % How wide is the map
StartPoint = [1;500];
EndPoint = [499;500];
Target = [0,5]; % The position of the target pole
precision = 0.01; % How precise is the map
RTsettings = struct('useFD',1,'stepSize',1);
pathlengthL = [];pathlengthR = [];
socialEnergyL = [];socialEnergyR = [];
TimeL = [];TimeR = [];
pathall = [];

%% Generate path
for i = 1:2 % lateral distance
    for j = 1:3 % front-back
        for k = 1:4 % 0:facing the participant; 90:right side
            socialImpact = 0;
            LD = 0.5 - 0.5*i;
            FB = j + 0.5;
            Blocker = [width/2+LD,FB];
            Blocker_Ori = 90*k - 90;
            EnergyMap = GetMap(width/precision+1,Target(2)/precision+1,Blocker/precision,Blocker_Ori,m,n,a,b,c,precision);
            EnergyMap = rescale(EnergyMap,0,1,'InputMin',0,'InputMax',th);
            SpeedMap = 1 - EnergyMap;
            T = fm(SpeedMap,StartPoint,[1 1],struct('implementation','C++','order',2));
            path = rayTrace(T,EndPoint,StartPoint,RTsettings);
            path2 = sortrows(path',2);
            path3 = interp1(path2(:,2),path2(:,1),1:470);
            path3(1) = (path3(2)-500)/2+500;
            newpath = [ones(470,1)*LD,ones(470,1)*FB,ones(470,1)*Blocker_Ori,[1:470]',path3'];
            pathall = [pathall;newpath];
        end
    end
end

pathall(:,5) = pathall(:,5) - 500;
pathall(:,5) = abs(pathall(:,5));
pathall(:,3) = pathall(:,3) * -1 + 450;
pathall(pathall(:,3)>=360,3) = pathall(pathall(:,3)>=360,3) - 360;
pathall = sortrows(pathall,[-1 2 3]);
dlmwrite('./results/path_multiple(social).csv',pathall);

%% IF MUST RIGHT
for i = 1:3 % lateral distance
    for j = 1:3 % front-back
        for k = 1:4 % 0:facing the participant; 90:right side
            socialImpact = 0;
            LD = 0.5*i - 1;
            FB = j + 0.5;
            Blocker = [width/2+LD,FB];
            Blocker_Ori = 90*k - 90;
            EnergyMap = GetMap(width/precision+1,Target(2)/precision+1,Blocker/precision,Blocker_Ori,m,n,a,b,c,precision);
            EnergyMap = rescale(EnergyMap,0,1,'InputMin',0,'InputMax',th);
            % Key Manipulation %
            EnergyMap(1:round(Blocker(1)/precision),FB/precision + 1) = 1;
            % Key Manipulation %
            SpeedMap = 1 - EnergyMap;
            T = fm(SpeedMap,StartPoint,[1 1],struct('implementation','C++','order',2));
            path = rayTrace(T,EndPoint,StartPoint,RTsettings);
            path2 = sortrows(path',2);
            path3 = interp1(path2(:,2),path2(:,1),1:470);
            path3(1) = (path3(2)-500)/2+500;
            h = diff(path3);
            l = sum(sqrt(h.^2+1));
            newpath = [ones(470,1)*LD,ones(470,1)*FB,ones(470,1)*Blocker_Ori,[1:470]',path3'];
            pathlengthR = [pathlengthR;[LD,FB,Blocker_Ori,l]];
            for ii = 1:length(newpath)
                socialImpact = socialImpact + EnergyMap(round(newpath(ii,5)),newpath(ii,4));
            end
            socialEnergyR = [socialEnergyR;[LD,FB,Blocker_Ori,socialImpact]];
            TimeR = [TimeR;[LD,FB,Blocker_Ori,T(500,500)]];
        end
    end
end

pathlengthR(:,3) = pathlengthR(:,3) * -1 + 450;
pathlengthR(pathlengthR(:,3)>=360,3) = pathlengthR(pathlengthR(:,3)>=360,3) - 360;
pathlengthR = sortrows(pathlengthR,[1 2 3]);

socialEnergyR(:,3) = socialEnergyR(:,3) * -1 + 450;
socialEnergyR(socialEnergyR(:,3)>=360,3) = socialEnergyR(socialEnergyR(:,3)>=360,3) - 360;
socialEnergyR = sortrows(socialEnergyR,[1 2 3]);

TimeR(:,3) = TimeR(:,3) * -1 + 450;
TimeR(TimeR(:,3)>=360,3) = TimeR(TimeR(:,3)>=360,3) - 360;
TimeR = sortrows(TimeR,[1 2 3]);

%% IF MUST LEFT
for i = 1:3 % lateral distance
    for j = 1:3 % front-back
        for k = 1:4 % 0:facing the participant; 90:right side
            socialImpact = 0;
            LD = 0.5*i - 1;
            FB = j + 0.5;
            Blocker = [width/2+LD,FB];
            Blocker_Ori = 90*k - 90;
            EnergyMap = GetMap(width/precision+1,Target(2)/precision+1,Blocker/precision,Blocker_Ori,m,n,a,b,c,precision);
            EnergyMap = rescale(EnergyMap,0,1,'InputMin',0,'InputMax',th);
            % Key Manipulation %
            EnergyMap(round(Blocker(1)/precision):1001,FB/precision + 1) = 1;
            % Key Manipulation %
            SpeedMap = 1 - EnergyMap;
            T = fm(SpeedMap,StartPoint,[1 1],struct('implementation','C++','order',2));
            path = rayTrace(T,EndPoint,StartPoint,RTsettings);
            path2 = sortrows(path',2);
            path3 = interp1(path2(:,2),path2(:,1),1:470);
            path3(1) = (path3(2)-500)/2+500;
            h = diff(path3);
            l = sum(sqrt(h.^2+1));
            newpath = [ones(470,1)*LD,ones(470,1)*FB,ones(470,1)*Blocker_Ori,[1:470]',path3'];
            pathlengthL = [pathlengthL;[LD,FB,Blocker_Ori,l]];
            for ii = 1:length(newpath)
                socialImpact = socialImpact + EnergyMap(round(newpath(ii,5)),newpath(ii,4));
            end
            socialEnergyL = [socialEnergyL;[LD,FB,Blocker_Ori,socialImpact]];
            TimeL = [TimeL;[LD,FB,Blocker_Ori,T(500,500)]];
        end
    end
end

pathlengthL(:,3) = pathlengthL(:,3) * -1 + 450;
pathlengthL(pathlengthL(:,3)>=360,3) = pathlengthL(pathlengthL(:,3)>=360,3) - 360;
pathlengthL = sortrows(pathlengthL,[1 2 3]);

socialEnergyL(:,3) = socialEnergyL(:,3) * -1 + 450;
socialEnergyL(socialEnergyL(:,3)>=360,3) = socialEnergyL(socialEnergyL(:,3)>=360,3) - 360;
socialEnergyL = sortrows(socialEnergyL,[1 2 3]);

TimeL(:,3) = TimeL(:,3) * -1 + 450;
TimeL(TimeL(:,3)>=360,3) = TimeL(TimeL(:,3)>=360,3) - 360;
TimeL = sortrows(TimeL,[1 2 3]);

%% Difference
pathlengthRatio = pathlengthL(:,4)./pathlengthR(:,4);
pathlengthDiff = pathlengthL(:,4) - pathlengthR(:,4);

socialEnergyRatio = socialEnergyL(:,4)./socialEnergyR(:,4);
socialEnergyDiff = socialEnergyL(:,4) - socialEnergyR(:,4);

TimeRatio = TimeL(:,4)./TimeR(:,4);
TimeDiff = TimeL(:,4) - TimeR(:,4);

realdata = dlmread('MultipleLR.csv');
realP = realdata(:,4);
% scatter(socialEnergy(:,4),realP)
% scatter(Time(:,4),realP)
% scatter(pathlength(:,5),realP)
% scatter(socialEnergy(:,5),realP)
% scatter(Time(:,5),realP)
% 1./(1+exp(a+b*x))  psychometric
% 1-exp(-1*(x/a)^b)  weibull

modelPrediction = [realP,socialEnergyRatio];
dlmwrite('./results/corM_s.csv',modelPrediction);