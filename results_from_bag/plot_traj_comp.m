%% Reference：
% 1. matlab - legend
% https://ww2.mathworks.cn/help/matlab/ref/legend.html

%% Code
clear;clc;close all;

bags = {struct('mode', 'real', 'name', 'thymio_real_with_obstacle_avoidance'), ...
            struct('mode', 'simu', 'name', 'thymio_simulation_navigation_with_obstacle_avoidance'),...
            struct('mode', 'real', 'name', 'thymio_real_without_obstacle')};

saveFileType = 'pdf'; % pdf/png

fontSize = 24;
fontSizeLegend = 24;
lineWidth = 2;

figure41 = figure(401);
set(figure41,'position',[0 0 1000 600]); % 0 0 1350 550

%base_folder = './results_from_bag';
base_folder = '.';
% real
bag_index = 1;
bagdata_folder = fullfile(base_folder, bags{bag_index}.mode, bags{bag_index}.name);
saveFileName = ['traj_' bags{bag_index}.name];
x = readmatrix([bagdata_folder '/' 'x_data.csv']);
y = readmatrix([bagdata_folder '/' 'y_data.csv']);
data_len = length(x);
plot(x(3:data_len), y(3:data_len), '-ro', 'LineWidth',lineWidth);
hold on;

% simu
bag_index = 2;
bagdata_folder = fullfile(base_folder, bags{bag_index}.mode, bags{bag_index}.name);
saveFileName = ['traj_' bags{bag_index}.name];
x = readmatrix([bagdata_folder '/' 'x_data.csv']);
y = readmatrix([bagdata_folder '/' 'y_data.csv']);
data_len = length(x);
plot(x(3:data_len), y(3:data_len), '-go', 'LineWidth',lineWidth);



xLabelName = 'x (m)';
yLabelName = 'y (m)';

% set(gca,'ytick',[],'yticklabel',[])
xlabel(xLabelName,'fontsize',fontSize,'fontname','Times New Roman','fontweight','bold');
ylabel(yLabelName,'fontsize',fontSize,'fontname','Times New Roman','fontweight','bold');
set(gca,'FontName','Times New Roman','fontSize',fontSize);
set(gca,'Xgrid','on');
set(gca,'Ygrid','on');
legend({'real','simu'}, 'Location','northwest','FontSize',fontSize);
title('Trajectory Comparison between Real and Simulation ');
tightfig;

%
switch saveFileType
    case 'pdf'
        saveas(gcf,[saveFileName '.pdf']); % pdf
    case 'png'
        saveas(gcf,saveFileName,'png'); % png
    otherwise
        disp('Type error !')
end