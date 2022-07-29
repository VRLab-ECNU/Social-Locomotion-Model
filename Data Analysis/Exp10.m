clc;clear;

num_clip = 40;

%% Calculate the MAE of each Clip
r = cell(1,num_clip);s = cell(1,num_clip);c = cell(1,num_clip);b = cell(1,num_clip);
mae = nan(num_clip,3);
for i = 1:num_clip
    clip = i;
    
    load(['Real Path/',num2str(clip),'.mat']);
    route_s = readmatrix(['Model Prediction/Social/Trial',num2str(clip),'.csv']);
    route_c = readmatrix(['Model Prediction/COMPANIAN/Trial',num2str(clip),'.csv']);
    route_b = readmatrix(['Model Prediction/N Body/Trial',num2str(clip),'.csv']);
    route_c = 100 * route_c;
    route_b(:,1) = [];
    route_b(1,:) = [];
    route_b = 100 * route_b;
    
    for j = 1:5
        if size(route{j},1) > 1
            r{i} = [r{i};route{j}];
            s{i} = [s{i};route_s(:,2*(j-1)+1:2*j)];
            c{i} = [c{i};route_c(:,2*(j-1)+1:2*j)];
            b{i} = [b{i};route_b(:,2*(j-1)+1:2*j)];
            
            m = max([size(route{j},1),size(route_s(:,2*(j-1)+1:2*j),1),size(route_c(:,2*(j-1)+1:2*j),1),size(route_b(:,2*(j-1)+1:2*j),1)]);
            
            for k = size(route{j},1)+1:m
                r{i} = [r{i};route{j}(end,:)];
            end
            for k = size(route_s(:,2*(j-1)+1:2*j),1)+1:m
                s{i} = [s{i};route_s(end,2*(j-1)+1:2*j)];
            end
            for k = size(route_c(:,2*(j-1)+1:2*j),1)+1:m
                c{i} = [c{i};route_c(end,2*(j-1)+1:2*j)];
            end
            for k = size(route_b(:,2*(j-1)+1:2*j),1)+1:m
                b{i} = [b{i};route_b(end,2*(j-1)+1:2*j)];
            end
        end
    end
    
    real = r{i}; social = s{i}; companian = c{i}; body = b{i};
    d = zeros(size(r{i},1),3);
    for j = 1:size(r{i},1)
        d(j,1) = norm(social(j,:)-real(j,:));
        d(j,2) = norm(companian(j,:)-real(j,:));
        d(j,3) = norm(body(j,:)-real(j,:));
    end
    mae(i,:) = mean(d,1);
end

mae_mean = mean(mae,1);