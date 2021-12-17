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
EndPoint = [769;500];
Target = [0,7.7]; % The position of the target pole
precision = 0.01; % How precise is the map
RTsettings = struct('useFD',1,'stepSize',1);
pathall = [];

%% Generate path
for i = 1:2 % lateral distance
    for j = 1:3 % front-back
        for k = 1:4 % 0:facing the participant; 90:right side
            socialImpact = 0;
            LD = 0.5 - 0.5*i;
            FB = j + 1.85;
            Blocker = [width/2+LD,FB];
            Blocker_Ori = 90*k - 90;
            EnergyMap = GetMap(width/precision+1,Target(2)/precision+1,Blocker/precision,Blocker_Ori,m,n,a,b,c,precision);
            EnergyMap = rescale(EnergyMap,0,1,'InputMin',0,'InputMax',th);
            SpeedMap = 1 - EnergyMap;
            T = fm(SpeedMap,StartPoint,[1 1],struct('implementation','C++','order',2));
            path = rayTrace(T,EndPoint,StartPoint,RTsettings);
            path2 = sortrows(path',2);
            path3 = interp1(path2(:,2),path2(:,1),70:700);
            newpath = [ones(631,1)*LD,ones(631,1)*FB,ones(631,1)*Blocker_Ori,[70:700]',path3'];
            pathall = [pathall;newpath];
        end
    end
end

pathall(:,5) = pathall(:,5) - 500;
pathall(:,5) = abs(pathall(:,5));
pathall(:,3) = pathall(:,3) * -1 + 450;
pathall(pathall(:,3)>=360,3) = pathall(pathall(:,3)>=360,3) - 360;
pathall = sortrows(pathall,[1 -2 3]);
dlmwrite('./results/path_real(social).csv',pathall);