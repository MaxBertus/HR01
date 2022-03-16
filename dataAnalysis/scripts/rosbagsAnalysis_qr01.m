clc
clear all
close all

%% Fixed roto-translation

p_map_vicon = [-0.25600 -0.25600 0]';
R_map_vicon = [0 1 0; 1 0 0; 0 0 -1];

p_marker_com = [-0.06 0 -0.025]';
%R_marker_com = [0 -1 0; -1 0 0; 0 0 -1];

%% Data import

% CROSS
%load('rosbag2_2022_02_22-15_40_00.mat')

% OTHER
% load('rosbag2_2022_02_22-16_04_59.mat')

traj = input('Insert trajectory to analyze: ');
num = input('Insert test number: ');

if traj == 0
    switch num % SQUARE 
        case 1
            load('rosbag2_2022_02_22-15_52_39.mat')
        case 2
            load('rosbag2_2022_02_22-15_56_11.mat')
        case 3
            load('rosbag2_2022_02_22-16_23_32.mat')
        case 4
            load('rosbag2_2022_02_22-16_26_47.mat')
        case 5
            load('rosbag2_2022_02_22-16_34_17.mat')
    end
else
     switch num % STEPS 
        case 1
            load('rosbag2_2022_02_22-16_39_23.mat')
        case 2
            load('rosbag2_2022_02_22-16_48_43.mat')
        case 3
            load('rosbag2_2022_02_22-16_53_44.mat')
        case 4
            load('rosbag2_2022_02_22-16_57_35.mat')
        case 5
            load('rosbag2_2022_02_22-17_01_08.mat')
     end
end

%% Data preparation
% Time alignment
time_samples = length(time);
time = time-time(1)*ones(time_samples,1);

%% VICON
% POSITION
% Unit of measure adjustment
p_marker_vicon_x = viconqr01qr01x_trans'./1e+03;
p_marker_vicon_y = viconqr01qr01y_trans'./1e+03;
p_marker_vicon_z = viconqr01qr01z_trans'./1e+03;

% NaN data replacement
p_marker_vicon_x = fillmissing(p_marker_vicon_x,'linear');
p_marker_vicon_y = fillmissing(p_marker_vicon_y,'linear');
p_marker_vicon_z = fillmissing(p_marker_vicon_z,'linear');

p_marker_vicon   = [p_marker_vicon_x; p_marker_vicon_y; p_marker_vicon_z];

% ATTITUDE
% Unit of measure adjustment
q_marker_vicon_w = viconqr01qr01w'./1e+03;
q_marker_vicon_x = viconqr01qr01x_rot'./1e+03;
q_marker_vicon_y = viconqr01qr01y_rot'./1e+03;
q_marker_vicon_z = viconqr01qr01z_rot'./1e+03;

% NaN data replacement
q_marker_vicon_w = fillmissing(q_marker_vicon_w,'linear');
q_marker_vicon_x = fillmissing(q_marker_vicon_x,'linear');
q_marker_vicon_y = fillmissing(q_marker_vicon_y,'linear');
q_marker_vicon_z = fillmissing(q_marker_vicon_z,'linear');

q_marker_vicon   = [q_marker_vicon_w; q_marker_vicon_x; q_marker_vicon_y; q_marker_vicon_z];

%%%

%% VIO
% POSITION
% NaN data replacement
p_com_map_vio_x = fmuvehicle_visual_odometryinx';
p_com_map_vio_y = fmuvehicle_visual_odometryiny';
p_com_map_vio_z = fmuvehicle_visual_odometryinz';

% (first datum setting)
fmuvehicle_visual_odometryinx_nNaN = fmuvehicle_visual_odometryinx((~isnan(fmuvehicle_visual_odometryinx)));
fmuvehicle_visual_odometryiny_nNaN = fmuvehicle_visual_odometryiny((~isnan(fmuvehicle_visual_odometryiny)));
fmuvehicle_visual_odometryinz_nNaN = fmuvehicle_visual_odometryinz((~isnan(fmuvehicle_visual_odometryinz)));
p_com_map_vio_x(1) = fmuvehicle_visual_odometryinx_nNaN(1);
p_com_map_vio_y(1) = fmuvehicle_visual_odometryiny_nNaN(1);
p_com_map_vio_z(1) = fmuvehicle_visual_odometryinz_nNaN(1);

p_com_map_vio_z = fillmissing(p_com_map_vio_z,'linear');
p_com_map_vio_y = fillmissing(p_com_map_vio_y,'linear');
p_com_map_vio_x = fillmissing(p_com_map_vio_x,'linear');

p_com_map_vio = [p_com_map_vio_x; p_com_map_vio_y; p_com_map_vio_z];

% ATTITUDE
q_com_map_vio_w = fmuvehicle_visual_odometryinq0';
q_com_map_vio_x = fmuvehicle_visual_odometryinq1';
q_com_map_vio_y = fmuvehicle_visual_odometryinq2';
q_com_map_vio_z = fmuvehicle_visual_odometryinq3';

% (first datum setting)
fmuvehicle_visual_odometryinq0_nNaN = fmuvehicle_visual_odometryinq0((~isnan(fmuvehicle_visual_odometryinq0)));
fmuvehicle_visual_odometryinq1_nNaN = fmuvehicle_visual_odometryinq1((~isnan(fmuvehicle_visual_odometryinq1)));
fmuvehicle_visual_odometryinq2_nNaN = fmuvehicle_visual_odometryinq2((~isnan(fmuvehicle_visual_odometryinq2)));
fmuvehicle_visual_odometryinq3_nNaN = fmuvehicle_visual_odometryinq3((~isnan(fmuvehicle_visual_odometryinq3)));

q_com_map_vio_w(1) = fmuvehicle_visual_odometryinq0_nNaN(1);
q_com_map_vio_x(1) = fmuvehicle_visual_odometryinq1_nNaN(1);
q_com_map_vio_y(1) = fmuvehicle_visual_odometryinq2_nNaN(1);
q_com_map_vio_z(1) = fmuvehicle_visual_odometryinq3_nNaN(1);

% q_com_map_vio_w(1) = 1;
% q_com_map_vio_x(1) = 0;
% q_com_map_vio_y(1) = 0;
% q_com_map_vio_z(1) = 0;

q_com_map_vio_w = fillmissing(q_com_map_vio_w,'linear');
q_com_map_vio_x = fillmissing(q_com_map_vio_x,'linear');
q_com_map_vio_y = fillmissing(q_com_map_vio_y,'linear');
q_com_map_vio_z = fillmissing(q_com_map_vio_z,'linear');

q_com_map_vio = [q_com_map_vio_w; q_com_map_vio_x; q_com_map_vio_y; q_com_map_vio_z];

%%%

%% EKF: POSITION
% NaN data replacement
p_com_map_ekf_x = fmuvehicle_odometryoutx';
p_com_map_ekf_y = fmuvehicle_odometryouty';
p_com_map_ekf_z = fmuvehicle_odometryoutz';

% (first datum setting)
fmuvehicle_odometryoutx_nNaN = fmuvehicle_odometryoutx((~isnan(fmuvehicle_odometryoutx)));
fmuvehicle_odometryouty_nNaN = fmuvehicle_odometryouty((~isnan(fmuvehicle_odometryouty)));
fmuvehicle_odometryoutz_nNaN = fmuvehicle_odometryoutz((~isnan(fmuvehicle_odometryoutz)));
p_com_map_ekf_x(1) = fmuvehicle_visual_odometryinx_nNaN(1);
p_com_map_ekf_y(1) = fmuvehicle_visual_odometryiny_nNaN(1);
p_com_map_ekf_z(1) = fmuvehicle_visual_odometryinz_nNaN(1);

p_com_map_ekf_x = fillmissing(p_com_map_ekf_x,'linear');
p_com_map_ekf_y = fillmissing(p_com_map_ekf_y,'linear');
p_com_map_ekf_z = fillmissing(p_com_map_ekf_z,'linear');

p_com_map_ekf = [p_com_map_ekf_x; p_com_map_ekf_y; p_com_map_ekf_z];

% EKF: ATTITUDE
q_com_map_ekf_w = fmuvehicle_odometryoutq0';
q_com_map_ekf_x = fmuvehicle_odometryoutq1';
q_com_map_ekf_y = fmuvehicle_odometryoutq2';
q_com_map_ekf_z = fmuvehicle_odometryoutq3';

% (first datum setting)
fmuvehicle_odometryoutq0_nNaN = fmuvehicle_odometryoutq0((~isnan(fmuvehicle_odometryoutq0)));
fmuvehicle_odometryoutq1_nNaN = fmuvehicle_odometryoutq1((~isnan(fmuvehicle_odometryoutq1)));
fmuvehicle_odometryoutq2_nNaN = fmuvehicle_odometryoutq2((~isnan(fmuvehicle_odometryoutq2)));
fmuvehicle_odometryoutq3_nNaN = fmuvehicle_odometryoutq3((~isnan(fmuvehicle_odometryoutq3)));

q_com_map_ekf_w(1) = fmuvehicle_odometryoutq0_nNaN(1);
q_com_map_ekf_x(1) = fmuvehicle_odometryoutq1_nNaN(1);
q_com_map_ekf_y(1) = fmuvehicle_odometryoutq2_nNaN(1);
q_com_map_ekf_z(1) = fmuvehicle_odometryoutq3_nNaN(1);


% q_com_map_ekf_w(1) = 1;
% q_com_map_ekf_x(1) = 0;
% q_com_map_ekf_y(1) = 0;
% q_com_map_ekf_z(1) = 0;

q_com_map_ekf_w = fillmissing(q_com_map_ekf_w,'linear');
q_com_map_ekf_x = fillmissing(q_com_map_ekf_x,'linear');
q_com_map_ekf_y = fillmissing(q_com_map_ekf_y,'linear');
q_com_map_ekf_z = fillmissing(q_com_map_ekf_z,'linear');

q_com_map_ekf = [q_com_map_ekf_w; q_com_map_ekf_x; q_com_map_ekf_y; q_com_map_ekf_z];

%% Reference frames alignment

p_marker_map_vio = zeros(3,time_samples);
p_marker_map_ekf = zeros(3,time_samples);
p_marker_vicon_vio = zeros(3,time_samples);
p_marker_vicon_ekf = zeros(3,time_samples);

for ii = 1:time_samples
    
    % Marker position expressed through VIO
    p_marker_map_vio(:,ii) = quat2rotm(q_com_map_vio(:,ii)')*p_marker_com + p_com_map_vio(:,ii);
    p_marker_map_ekf(:,ii) = quat2rotm(q_com_map_ekf(:,ii)')*p_marker_com + p_com_map_ekf(:,ii);
    
    % Marker position expressed through VIO in Vicon frame
    p_marker_vicon_vio(:,ii) = R_map_vicon*p_marker_map_vio(:,ii)+p_map_vicon;
    p_marker_vicon_ekf(:,ii) = R_map_vicon*p_marker_map_ekf(:,ii)+p_map_vicon;

end

%% Errors computation

e_vcn_vio = (p_marker_vicon-p_marker_vicon_vio);
e_vcn_ekf = (p_marker_vicon-p_marker_vicon_ekf);
e_ekf_vio = (p_marker_vicon_ekf-p_marker_vicon_vio);

% Offset compensation
t_start = 10;
[~,t_start_idx] = (min(abs(time - t_start)));

e_vcn_vio = e_vcn_vio-kron([mean(e_vcn_vio(1,1:t_start_idx)) mean(e_vcn_vio(2,1:t_start_idx)) mean(e_vcn_vio(3,1:t_start_idx))]',ones(1,time_samples));
e_vcn_ekf = e_vcn_ekf-kron([mean(e_vcn_ekf(1,1:t_start_idx)) mean(e_vcn_ekf(2,1:t_start_idx)) mean(e_vcn_ekf(3,1:t_start_idx))]',ones(1,time_samples));
e_ekf_vio = e_ekf_vio-kron([mean(e_ekf_vio(1,1:t_start_idx)) mean(e_ekf_vio(2,1:t_start_idx)) mean(e_ekf_vio(3,1:t_start_idx))]',ones(1,time_samples));

e_vcn_vio_norm = normOverTime(e_vcn_vio);
e_vcn_ekf_norm = normOverTime(e_vcn_ekf); 
e_ekf_vio_norm = normOverTime(e_ekf_vio);

%% Results plot

linewidth = 1.5;
fontsize = 16;

% 3D plot
figure("Position",[200 200 600 400])
plot3(p_marker_vicon(1,:),p_marker_vicon(2,:),p_marker_vicon(3,:),'k','LineWidth', linewidth);
hold on
plot3(p_marker_vicon_vio(1,:),p_marker_vicon_vio(2,:),p_marker_vicon_vio(3,:),'b','LineWidth', linewidth);
hold on
plot3(p_marker_vicon_ekf(1,:),p_marker_vicon_ekf(2,:),p_marker_vicon_ekf(3,:),'r','LineWidth', linewidth);
grid on
set(gca,'TickLabelInterpreter','latex','FontSize',fontsize);
xlabel('$x$','Interpreter','latex','FontSize',fontsize);
ylabel('$y$','Interpreter','latex','FontSize',fontsize);
zlabel('$z$','Interpreter','latex','FontSize',fontsize);
legend('vcn','vio','efk','Interpreter','latex','FontSize',fontsize);

% Absolute positions
figure("Position",[1000 200 1600 400])
subplot(1,3,1)
plot(time,p_marker_vicon(1,:),'k','LineWidth', linewidth);
hold on
% plot(time,p_marker_vicon_vio(1,:),'b','LineWidth', linewidth);
% hold on
% plot(time,p_marker_vicon_ekf(1,:),'r','LineWidth', linewidth);
grid on
set(gca,'TickLabelInterpreter','latex','FontSize',fontsize);
xlabel('time','Interpreter','latex','FontSize',fontsize);
ylabel('$p_x$','Interpreter','latex','FontSize',fontsize);
subplot(1,3,2)
plot(time,p_marker_vicon(2,:),'k','LineWidth', linewidth);
hold on
% plot(time,p_marker_vicon_vio(2,:),'b','LineWidth', linewidth);
% hold on
% plot(time,p_marker_vicon_ekf(2,:),'r','LineWidth', linewidth);
grid on
set(gca,'TickLabelInterpreter','latex','FontSize',fontsize);
xlabel('time','Interpreter','latex','FontSize',fontsize);
ylabel('$p_y$','Interpreter','latex','FontSize',fontsize);
subplot(1,3,3)
plot(time,p_marker_vicon(3,:),'k','LineWidth', linewidth);
hold on
% plot(time,p_marker_vicon_vio(3,:),'b','LineWidth', linewidth);
% hold on
% plot(time,p_marker_vicon_ekf(3,:),'r','LineWidth', linewidth);
grid on
set(gca,'TickLabelInterpreter','latex','FontSize',fontsize);
xlabel('time','Interpreter','latex','FontSize',fontsize);
ylabel('$p_z$','Interpreter','latex','FontSize',fontsize);
legend('vcn','vio','efk','Interpreter','latex','FontSize',fontsize, 'Location', 'southeast');

% % Errors
% figure("Position",[1000 800 1600 400])
% subplot(1,3,1)
% plot(time,e_vcn_vio(1,:),'k','LineWidth', linewidth);
% hold on
% plot(time,e_vcn_ekf(1,:),'b','LineWidth', linewidth);
% hold on
% plot(time,e_ekf_vio(1,:),'r','LineWidth', linewidth);
% grid on
% set(gca,'TickLabelInterpreter','latex','FontSize',fontsize);
% xlabel('time','Interpreter','latex','FontSize',fontsize);
% ylabel('$e_x$','Interpreter','latex','FontSize',fontsize);
% subplot(1,3,2)
% plot(time,e_vcn_vio(2,:),'k','LineWidth', linewidth);
% hold on
% plot(time,e_vcn_ekf(2,:),'b','LineWidth', linewidth);
% hold on
% plot(time,e_ekf_vio(2,:),'r','LineWidth', linewidth);
% grid on
% set(gca,'TickLabelInterpreter','latex','FontSize',fontsize);
% xlabel('time','Interpreter','latex','FontSize',fontsize);
% ylabel('$e_y$','Interpreter','latex','FontSize',fontsize);
% subplot(1,3,3)
% plot(time,e_vcn_vio(3,:),'k','LineWidth', linewidth);
% hold on
% plot(time,e_vcn_ekf(3,:),'b','LineWidth', linewidth);
% hold on
% plot(time,e_ekf_vio(3,:),'r','LineWidth', linewidth);
% grid on
% set(gca,'TickLabelInterpreter','latex','FontSize',fontsize);
% xlabel('time','Interpreter','latex','FontSize',fontsize);
% ylabel('$e_z$','Interpreter','latex','FontSize',fontsize);
% legend('vcn-vio','vcn-ekf','ekf-vio','Interpreter','latex','FontSize',fontsize, 'Location', 'northeast');

% %% Indeces computation
% 
% if num ~= 1
%     load('sequences_QR01.mat')
% end
% 
% if traj == 0
%     sequenceSquare(1,1,num) = find(round(time,4)==input('A start '));
%     sequenceSquare(2,1,num) = find(round(time,4)==input('A end '));
%          
%     sequenceSquare(1,2,num) = find(round(time,4)==input('B start '));
%     sequenceSquare(2,2,num) = find(round(time,4)==input('B end '));
%          
%     sequenceSquare(1,3,num) = find(round(time,4)==input('C start '));
%     sequenceSquare(2,3,num) = find(round(time,4)==input('C end '));
%         
%     sequenceSquare(1,4,num) = find(round(time,4)==input('D start '));
%     sequenceSquare(2,4,num) = find(round(time,4)==input('D end '));
%     save('sequences_QR01.mat', 'sequenceSquare')
% else
%     sequenceSteps(1,1,num) = find(round(time,4)==input('Ascent start '));
%     sequenceSteps(2,1,num) = find(round(time,4)==input('Ascent end '));
%          
%     sequenceSteps(1,2,num) = find(round(time,4)==input('Descent start '));
%     sequenceSteps(2,2,num) = find(round(time,4)==input('Descent end '));
%     save('sequences_QR01.mat', 'sequenceSteps','-append')
% end

%% MEAN, MEDIAN & STD 
load('sequences_QR01.mat')

if traj == 0
    sequences = sequenceSquare;
else
    sequences = sequenceSteps;
end

for ax=1:3
    disp("AXES " + num2str(ax))
    for i=1:size(sequences,2)
        phase = sequences(1,i,num):sequences(2,i,num);

        mean_vcn_vio(i) = mean(e_vcn_vio(ax,phase));
        mean_vcn_ekf(i) = mean(e_vcn_ekf(ax,phase));
        
        median_vcn_vio(i) = median(e_vcn_vio(ax,phase));
        median_vcn_ekf(i) = median(e_vcn_ekf(ax,phase));
        
        std_vcn_vio(i) = std(e_vcn_vio(ax,phase));
        std_vcn_ekf(i) = std(e_vcn_ekf(ax,phase));

        quart_vcn_vio = quantile(e_vcn_vio(ax,phase),4);
        quart_vcn_ekf = quantile(e_vcn_ekf(ax,phase),4);
        iqr_vcn_vio(i) = quart_vcn_vio(2) - quart_vcn_vio(1);
        iqr_vcn_ekf(i) = quart_vcn_ekf(2) - quart_vcn_ekf(1);
        
        range_vcn_vio(i) = range(e_vcn_vio(ax,phase));
        range_vcn_ekf(i) = range(e_vcn_ekf(ax,phase));            


        disp("Square phase " + num2str(i) + ":" + newline + ...
            "- mean "   + num2str(mean_vcn_vio(i))   + " / " + num2str(mean_vcn_ekf(i))   + " m" + newline +...
            "- median " + num2str(median_vcn_vio(i)) + " / " + num2str(median_vcn_ekf(i)) + " m" + newline +...
            "- std "    + num2str(std_vcn_vio(i))    + " / " + num2str(std_vcn_ekf(i))    + " m" + newline + ...
            "- iqr "    + num2str(iqr_vcn_vio(i))    + " / " + num2str(iqr_vcn_ekf(i))    + " m" + newline + ...
            "- range "  + num2str(range_vcn_vio(i))  + " / " + num2str(range_vcn_ekf(i))  + newline ...
            );
    end
    disp("%---------------------%")
end