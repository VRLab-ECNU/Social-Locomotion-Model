clc;clear;

%% Add a New Path
oldpath = path;
path(oldpath,'functions');

%% Import Data
load('data/fit_parameters.mat');
load('data/condition.mat');
load('data/speed.mat');

%% Setting Initial Values
m1 = x(1);
n1 = x(2);
m2 = x(3);
n2 = x(4);
a = 0.24;
b = 0.12;
c = x(5);
th = x(6);
ncf = x(7);
width = 13.5; % How wide is the map
depth = 13.5; % How deep is the map
precision = 0.01; % How precise is the map
fps = 30; timestep = 1/fps;
minStep = speed/precision * timestep; % cm/frame
timepoints = 500;
refresh = fps;
RTsettings = struct('useFD',1,'stepSize',1);

%% Reading Conditions
num_trial = size(condition,1);
num_ava = floor(size(condition,2)/6); % Number of avatars in the scene
isStatic = nan(num_trial,num_ava); % 1: Static; 0: Movable
start_pos = nan(num_trial,num_ava,2);
end_pos = nan(num_trial,num_ava,2);
ori = nan(num_trial,num_ava); % 90: facing right; 0: facing buttom  
for i = 1:num_trial
    for j = 1:num_ava
        isStatic(i,j) = condition(i,6*(j-1)+1); 
        ori(i,j) = condition(i,6*j);
        if condition(i,6*(j-1)+1) == 1 % The avatar shall not move
            start_pos(i,j,1) = condition(i,6*(j-1)+2)/precision;
            start_pos(i,j,2) = condition(i,6*(j-1)+3)/precision;
            end_pos(i,j,:) = start_pos(i,j,:);
        else
            start_pos(i,j,1) = condition(i,6*(j-1)+2)/precision;
            start_pos(i,j,2) = condition(i,6*(j-1)+3)/precision;
            end_pos(i,j,1) = condition(i,6*(j-1)+4)/precision;
            end_pos(i,j,2) = condition(i,6*(j-1)+5)/precision;
        end
    end
end

for i = 1:num_trial
    %% Generate Path
    pos = squeeze(start_pos(i,:,:));
    deg = ori(i,:);
    static = isStatic(i,:);
    pos_all = cell(1,num_ava);
    for j = 1:num_ava
        pos_all{j} = pos(j,:);
    end

    while sum(static) < num_ava
        pos_temp = cell(1,num_ava);
        
        for j = 1:num_ava
            if static(j) == 0
                EnergyMap = zeros(depth/precision+3,width/precision+3);
                StartPoint = pos(j,:);
                EndPoint = squeeze(end_pos(i,j,:))';
                
                for k = 1:num_ava
                    if j == k
                        continue;
                    end
                    if static(k) == 0 % Bypassing moving people
                        BlockerStart = pos(k,:);
                        Blocker_Ori = deg(k);
                        Ori = deg(j);
                        
                        % Calculate the closest distance
                        subPath = zeros(timepoints,2);
                        avaPath = zeros(timepoints,2);
                        for time = 1:timepoints
                            subPath(time,:) = StartPoint + [-cosd(Ori),sind(Ori)]*(time-1)*speed(i,j)*timestep/precision;
                            avaPath(time,:) = BlockerStart + [-cosd(Blocker_Ori),sind(Blocker_Ori)]*(time-1)*speed(i,k)*timestep/precision;
                        end
                        d = subPath - avaPath;
                        d1 = (d(:,1).^2 + d(:,2).^2).^0.5;
                        minD = inf;
                        for l = 1:timepoints
                            if d1(l) < minD
                                minD = d1(l);
                                min_n = l;
                            end
                        end
                        
                        Blocker = avaPath(min_n,:);
                        EnergyMap = EnergyMap + getMap(depth/precision+3,width/precision+3,Blocker+[2,2],-Blocker_Ori-90,EndPoint+[2,2],m1,n1,m2,n2,a,b,c,precision,ncf);
                    else % Bypassing static people
                        Blocker = pos(k,:);
                        Blocker_Ori = deg(k);
                        EnergyMap = EnergyMap + getMap(depth/precision+3,width/precision+3,Blocker+[2,2],-Blocker_Ori-90,EndPoint+[2,2],m1,n1,m2,n2,a,b,c,precision,ncf);
                    end
                end

                pos_sub = [round(pos(j,1)),round(pos(j,2))];
                field = EnergyMap(pos_sub(1)+2,pos_sub(2)+2);
                
                if field > th
                    EnergyMap = rescale(EnergyMap,0,1,'InputMin',0,'InputMax',field);
                else
                    EnergyMap = rescale(EnergyMap,0,1,'InputMin',0,'InputMax',th);
                end
                %surf(EnergyMap,'LineStyle','none');
                
                SpeedMap = 1 - EnergyMap;
                % Can't go outside
                SpeedMap(1,:) = 0;SpeedMap(end,:) = 0;SpeedMap(:,1) = 0;SpeedMap(:,end) = 0;
                
                T = fm(SpeedMap,flip(StartPoint+[2,2])',[1 1],struct('implementation','C++','order',2));
                path_ = rayTrace(T,flip(EndPoint+[2,2])',flip(StartPoint+[2,2])',RTsettings);
                if size(path_,2) > 1000
                    num = 1;
                    while num <= size(path_,2) && norm((path_(:,num)'-2)-pos(j,:)) >= minStep(i,j)
                        num = num + 1;
                    end
                    path_ = path_(:,1:num);
                end
                path_ = [StartPoint;flip(path_,2)'-2];
                l = arclength(path_(:,1),path_(:,2));
                path_ = interparc(round(l/minStep(i,j))+1,path_(:,1),path_(:,2));
                
                if size(path_,1) > refresh+1 % Haven't reach target
                    pos_temp{j} = path_(1:refresh+1,:);
                else
                    pos_temp{j} = path_;
                end 
            end
        end
        
        for j = 1:num_ava
            if static(j) == 0
                if size(pos_temp{j},1) > 1
                    pos_all{j} = [pos_all{j};pos_temp{j}(2,:)];
                    pos(j,:) = pos_temp{j}(2,:);
                    deg(j) = getAlpha(pos_temp{j}(2,:)-pos_temp{j}(1,:));
                else
                    pos_all{j} = [pos_all{j};pos_temp{j}];
                    pos(j,:) = pos_temp{j};
                    static(j) = 1;
                end
            end
        end
        
        for j = 2:refresh
            flag = true;
            for k = 1:num_ava
                if j > size(pos_temp{k},1)-1
                    static(k) = 1;
                end
                if static(k) == 0
                    field = 0;
                    for l = 1:num_ava
                        if k ~= l
                            pos_sub = pos_temp{k}(j+1,:);
                            if static(l) == 1 || j > size(pos_temp{l},1)-1
                                pos_ava = pos(l,:);
                                field = field + getField(pos_sub,pos_ava,-deg(l)-90,squeeze(end_pos(i,k,:))',m1,n1,m2,n2,a,b,c,precision,ncf);
                            else
                                pos_ava = pos_temp{l}(j+1,:);
                                field = field + getField(pos_sub,pos_ava,-getAlpha(pos_ava-pos_temp{l}(j,:))-90,squeeze(end_pos(i,k,:))',m1,n1,m2,n2,a,b,c,precision,ncf);
                            end
                        end
                    end
                    if field >= th
                        flag = false;
                        break;
                    end
                end
            end
            
            if flag == false
                break;
            end
            
            for k = 1:num_ava
                if static(k) == 0
                    pos_all{k} = [pos_all{k};pos_temp{k}(j+1,:)];
                    pos(k,:) = pos_temp{k}(j+1,:);
                    deg(k) = getAlpha(pos_temp{k}(j+1,:)-pos_temp{k}(j,:));
                end
            end
        end
    end
    
    %% Export Model Prediction
    save(['results/drone/',num2str(i),'.mat'],'pos_all');
    
    len = 0;
    for j = 1:num_ava
        if size(pos_all{j},1) > len
            len = size(pos_all{j},1);
        end
    end
    
    data = nan(len,2*num_ava);
    for j = 1:num_ava
        l = size(pos_all{j},1);
        data(1:l,2*j-1:2*j) = pos_all{j};
        for k = l+1:len
            data(k,2*j-1:2*j) = data(l,2*j-1:2*j);
        end
    end
    
    csvwrite(['results/drone/',num2str(i),'.csv'],data);
    
    %% Process Output
    disp(['Trial ',num2str(i),'/',num2str(num_trial),' completed']);
end

%% Restore the Original Path
path(oldpath);