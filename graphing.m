clc; clear
files = dir('./data/trap_*.mat');

hold on; grid on;

for i = 1:length(files)
    file_path = fullfile({files(i).folder}, {files(i).name});

    load(char(file_path));
    fprintf('Loaded file: %s\n', files(i).name);

    if i == 1
        plot(traj.x, traj.y)
    end

    plot(state_out(:,1),state_out(:,2))

end

xlabel('x-pos [m]')
ylabel('y-pos [m]')
title('Controller Trajectories for Trapezoidal Speed Tracking')
legend({'desired traj','FH-LQG','IH-LQG','PD'})