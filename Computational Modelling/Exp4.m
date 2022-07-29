clc;clear;
addpath('./functions');
load('./data/fit_parameters.mat');
m1 = x(1);
n1 = x(2);
m2 = x(3);
n2 = x(4);
a = 0.285;
b = 0.175;
c = x(5);
th = x(6);
ncf = x(7);
StartPoint = [1;5];
EndPoint = [505;2];
Target = [0,506]; % The position of the target pole
Blocker = [0,256]; % The position of the blocking avatar
precision = 0.01; % How precise is the map
width = 200; % How wide is the map
RTsettings = struct('useFD',1,'stepSize',1);
pathall = [];
pathlength = [];
socialEnergy = [];
Time = [];
MLD = [];

for i = 1:12 % 0:facing the participant; 90:right side
    socialImpact = 0;
    Blocker_Ori = 30*i - 30;
    EnergyMap = GetMap(width+1,Target(2)+1,Blocker,Blocker_Ori,Target,m1,n1,m2,n2,a,b,c,precision,ncf);
    EnergyMap = rescale(EnergyMap,0,1,'InputMin',0,'InputMax',th);
    SpeedMap = 1 - EnergyMap;
    T = fm(SpeedMap,StartPoint,[1 1],struct('implementation','C++','order',2));
    path = rayTrace(T,EndPoint,StartPoint,RTsettings);
    path2 = sortrows(path',2);
    path2(:,2) = path2(:,2)-6;
    path2 = path2(path2(:,2)>0,:);
    path3 = interp1(path2(:,2),path2(:,1),1:470);
    h = diff(path3);
    l = sum(sqrt(h.^2+1));
    newpath = [ones(470,1)*Blocker_Ori,[1:470]',path3'];
    pathall = [pathall;newpath];
    pathlength = [pathlength;[Blocker_Ori,l]];
    MLD = [MLD;[Blocker_Ori,max(path2(:,1))]];
    for j = 1:length(newpath)
        socialImpact = socialImpact + EnergyMap(round(newpath(j,3)),newpath(j,2));
    end
    socialEnergy = [socialEnergy;[Blocker_Ori,socialImpact]];
    Time = [Time;[Blocker_Ori,T(2,end)]];
end

pathall(:,1) = pathall(:,1) * -1 + 450;
pathall(pathall(:,1)>=360,1) = pathall(pathall(:,1)>=360,1) - 360;
pathall = sortrows(pathall,1);

pathlength(:,1) = pathlength(:,1) * -1 + 450;
pathlength(pathlength(:,1)>=360,1) = pathlength(pathlength(:,1)>=360,1) - 360;
pathlength = sortrows(pathlength,1);

socialEnergy(:,1) = socialEnergy(:,1) * -1 + 450;
socialEnergy(socialEnergy(:,1)>=360,1) = socialEnergy(socialEnergy(:,1)>=360,1) - 360;
socialEnergy = sortrows(socialEnergy,1);

Time(:,1) = Time(:,1) * -1 + 450;
Time(Time(:,1)>=360,1) = Time(Time(:,1)>=360,1) - 360;
Time = sortrows(Time,1);

MLD(:,1) = MLD(:,1) * -1 + 450;
MLD(MLD(:,1)>=360,1) = MLD(MLD(:,1)>=360,1) - 360;
MLD = sortrows(MLD,1);

for i = 1:12
    Blocker_Ori = 30*i - 30;
    if Blocker_Ori <= 180
        Another_Ori = 180 - Blocker_Ori;
    else
        Another_Ori = 540 - Blocker_Ori;
    end
    pathlength(i,3) = pathlength(pathlength(:,1) == Another_Ori,2);
    socialEnergy(i,3) = socialEnergy(socialEnergy(:,1) == Another_Ori,2);
    Time(i,3) = Time(Time(:,1) == Another_Ori,2);
end

pathlengthRatio = pathlength(:,3)./pathlength(:,2);
pathlengthDiff = pathlength(:,3) - pathlength(:,2);

socialEnergyRatio = socialEnergy(:,3)./socialEnergy(:,2);
socialEnergyDiff = socialEnergy(:,3) - socialEnergy(:,2);

TimeRatio = Time(:,3)./Time(:,2);
TimeDiff = Time(:,3) - Time(:,2);

realdata = dlmread('data/PLR.csv');
realP = realdata(:,2);

a = 19.25; b = -19.6;
modelP = 1./(1+exp(a+b*socialEnergyRatio));
modelPrediction = [realP,modelP];
