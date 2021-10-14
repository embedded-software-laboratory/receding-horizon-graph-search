function commonroad = commonroad_lanelets()

% commonroad_data = readstruct('LabMapCommonRoad.xml');
% % % save('commonroad_data.mat','commonroad_data')

load('commonroad_data.mat')

Nlanelets = length(horzcat(commonroad_data.lanelet.idAttribute));
commonroad_left_x = cell(1,Nlanelets);
commonroad_left_y = cell(1,Nlanelets);
commonroad_right_x = cell(1,Nlanelets);
commonroad_right_y = cell(1,Nlanelets);

for i = 1: Nlanelets
    commonroad_left_x{i} = horzcat(commonroad_data.lanelet(i).leftBound.point.x);
    commonroad_left_y{i}= horzcat(commonroad_data.lanelet(i).leftBound.point.y);
    commonroad_right_x{i} = horzcat(commonroad_data.lanelet(i).rightBound.point.x);
    commonroad_right_y{i} = horzcat(commonroad_data.lanelet(i).rightBound.point.y);
    commonroad = {commonroad_left_x;commonroad_left_y; commonroad_right_x;commonroad_right_y};
    
end


end