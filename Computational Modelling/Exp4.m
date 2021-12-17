clc;clear;
addpath('./functions');
load('./data/fit_parameters.mat');
m = x(1);
n = x(2);
a = 0.24;
b = 0.12;
c = x(3);
th = x(4)*10;
StartPoint = [1;1];
EndPoint = [499;2];
Target = [0,5]; % The position of the target pole
Blocker = [0,2.5]; % The position of the blocking avatar
precision = 0.01; % How precise is the map
width = 2; % How wide is the map
RTsettings = struct('useFD',1,'stepSize',1);
pathall = [];
pathlength = [];
socialEnergy = [];
Time = [];
MLD = [];

for i = 1:12 % 0:facing the participant; 90:right side
    socialImpact = 0;
    Blocker_Ori = 30*i - 30;
    EnergyMap = GetMap(width/precision+1,Target(2)/precision+1,Blocker/precision,Blocker_Ori,m,n,a,b,c,precision);
    EnergyMap = rescale(EnergyMap,0,1,'InputMin',0,'InputMax',th);
    SpeedMap = 1 - EnergyMap;
    T = fm(SpeedMap,StartPoint,[1 1],struct('implementation','C++','order',2));
    path = rayTrace(T,EndPoint,StartPoint,RTsettings);
    path2 = sortrows(path',2);
    path3 = interp1(path2(:,2),path2(:,1),1:470);
    path3(1) = path3(2)/2;
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
    Time = [Time;[Blocker_Ori,T(1,end)]];
end

pathall(:,1) = pathall(:,1) * -1 + 450;
pathall(pathall(:,1)>=360,1) = pathall(pathall(:,1)>=360,1) - 360;
pathall = sortrows(pathall,1);
dlmwrite('./results/path.csv',pathall);

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
dlmwrite('./results/MLD.csv',MLD(:,2))

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

realP = [0.208 ; 0.196 ; 0.28 ; 0.456 ; 0.7253 ; 0.808 ; 0.856 ; 0.89156 ; 0.896 ; 0.684 ; 0.26 ; 0.196];
% scatter(pathlength(:,4),realP)
% scatter(socialEnergy(:,4),realP)
% scatter(Time(:,4),realP)
% scatter(pathlength(:,5),realP)
% scatter(socialEnergy(:,5),realP)
% scatter(Time(:,5),realP)
% 1./(1+exp(a+b*x))  psychometric
% 1-exp(-1*(x/a)^b)  weibull

modelPrediction = [realP,modelP];
dlmwrite('./results/cor.csv',modelPrediction);