clc;clear;
addpath('./functions');
load('data/fit_parameters.mat');
m1 = x(1);
n1 = x(2);
m2 = x(3);
n2 = x(4);
a = 0.24;
b = 0.12;
c = x(5);
th = x(6);
ncf = x(7);
width = 10; % How wide is the map
StartPoint = [1;500];
EndPoint = [499;500];
Target = [0,5]; % The position of the target pole
BlockerL_Ori = 90; 
BlockerR_Ori = 270;
precision = 0.01; % How precise is the map
RTsettings = struct('useFD',1,'stepSize',1);
pathlengthL = [];pathlengthR = [];
socialEnergyL = [];socialEnergyR = [];
TimeL = [];TimeR = [];

%% IF MUST LEFT
for i = 1:5
    LD = i*0.2 - 0.8;
    for IPD = 2:4
        socialImpact = 0;
        BlockerL = [width/2 + LD , 2.5];
        BlockerR = [width/2 + LD + IPD , 2.5];
        EnergyMap1 = GetMap(width/precision+1,Target(2)/precision+1,BlockerL/precision,BlockerL_Ori,Target/precision,m1,n1,m2,n2,a,b,c,precision,ncf);
        EnergyMap2 = GetMap(width/precision+1,Target(2)/precision+1,BlockerR/precision,BlockerR_Ori,Target/precision,m1,n1,m2,n2,a,b,c,precision,ncf);
        EnergyMap = EnergyMap1 + EnergyMap2;
        EnergyMap = rescale(EnergyMap,0,1,'InputMin',0,'InputMax',th);
        % Key manipulation %
        EnergyMap(round(BlockerL(1)/precision):round(BlockerR(1)/precision),251) = 1;
        % Key manipulation %
        SpeedMap = 1 - EnergyMap;
        T = fm(SpeedMap,StartPoint,[1 1],struct('implementation','C++','order',2));
        path = rayTrace(T,EndPoint,StartPoint,RTsettings);
        path2 = sortrows(path',2);
        path3 = interp1(path2(:,2),path2(:,1),1:470);
        path3(1) = (path3(2)-500)/2+500;
        h = diff(path3);
        l = sum(sqrt(h.^2+1));
        pathlengthL = [pathlengthL;[LD,IPD,l]];
        for j = 1:length(path)-1
            socialImpact = socialImpact + EnergyMap(round(path(1,j)),round(path(2,j)));
        end
        socialEnergyL = [socialEnergyL;[LD,IPD,socialImpact]];
        TimeL = [TimeL;[LD,IPD,T(500,500)]];
    end
end

%% IF MUST RIGHT
for i = 1:5
    LD = i*0.2 - 0.8;
    for IPD = 2:4
        socialImpact = 0;
        BlockerL = [width/2 + LD , 2.5];
        BlockerR = [width/2 + LD + IPD , 2.5];
        EnergyMap1 = GetMap(width/precision+1,Target(2)/precision+1,BlockerL/precision,BlockerL_Ori,Target/precision,m1,n1,m2,n2,a,b,c,precision,ncf);
        EnergyMap2 = GetMap(width/precision+1,Target(2)/precision+1,BlockerR/precision,BlockerR_Ori,Target/precision,m1,n1,m2,n2,a,b,c,precision,ncf);
        EnergyMap = EnergyMap1 + EnergyMap2;
        EnergyMap = rescale(EnergyMap,0,1,'InputMin',0,'InputMax',th);
        % Key manipulation %
        EnergyMap(1:round(BlockerL(1)/precision),251) = 1;
        EnergyMap(round(BlockerR(1)/precision:1001),251) = 1;
        % Key manipulation %
        SpeedMap = 1 - EnergyMap;
        T = fm(SpeedMap,StartPoint,[1 1],struct('implementation','C++','order',2));
        path = rayTrace(T,EndPoint,StartPoint,RTsettings);
        path2 = sortrows(path',2);
        path3 = interp1(path2(:,2),path2(:,1),1:470);
        path3(1) = (path3(2)-500)/2+500;
        h = diff(path3);
        l = sum(sqrt(h.^2+1));
        pathlengthR = [pathlengthR;[LD,IPD,l]];
        for j = 1:length(path)-1
            socialImpact = socialImpact + EnergyMap(round(path(1,j)),round(path(2,j)));
        end
        socialEnergyR = [socialEnergyR;[LD,IPD,socialImpact]];
        TimeR = [TimeR;[LD,IPD,T(500,500)]];
    end
end

pathlengthRatio = pathlengthL(:,3)./pathlengthR(:,3);
pathlengthDiff = pathlengthL(:,3) - pathlengthR(:,3);

socialEnergyRatio = socialEnergyL(:,3)./socialEnergyR(:,3);
socialEnergyDiff = socialEnergyL(:,3) - socialEnergyR(:,3);

TimeRatio = TimeL(:,3)./TimeR(:,3);
TimeDiff = TimeL(:,3) - TimeR(:,3);

realdata = dlmread('data/finaldata.csv');
realP = realdata(:,3);

a = 5.647; b = -6.143;
model = 1./(1+exp(a+b*socialEnergyRatio));
modelPrediction = [realP,model];