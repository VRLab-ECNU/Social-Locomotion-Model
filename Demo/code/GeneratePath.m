function path = GeneratePath(mapWidth,Start,Target,Blocker,Blocker_Ori)
% Start = [5,0]; % The position of the start point
% Target = [5,5]; % The position of the end point
% mapWidth = 10; % How wide is the map
% Blocker = [5,2.5]; % The position of the blocking avatar
% Blocker_Ori = 0; % The orientation of the blocking avatar (deg)
addpath('../functions');
m1 = 0.438;
n1 = 0.63;
m2 = 0.321;
n2 = 0.856;
a = 0.24;
b = 0.12;
c = 1.43;
th = 10.18;
ncf = 2;
StartPoint = [Start(2)*100;Start(1)*100];
EndPoint = [Target(2)*100;Target(1)*100];
precision = 0.01; % How precise is the map
RTsettings = struct('useFD',1,'stepSize',1);
EnergyMap = GetMap(mapWidth/precision+1,Target(2)/precision+1,Blocker/precision,Blocker_Ori,EndPoint',m1,n1,m2,n2,a,b,c,precision,ncf);
EnergyMap = rescale(EnergyMap,0,1,'InputMin',0,'InputMax',th);
SpeedMap = 1 - EnergyMap;
T = fm(SpeedMap,StartPoint,[1 1],struct('implementation','C++','order',2));
path = rayTrace(T,EndPoint,StartPoint,RTsettings);
path = flip(path');
end