%% 1. 讀取資料
% 請確保路徑正確
real_data = readtable(['/home/cyc/campus_ws/mpcdata/real_delay0.4.csv']);
back_data = readtable('/home/cyc/campus_ws/src/mpc_4state/src/0414.csv');
sim_data  = readtable(['/home/cyc/campus_ws/mpcdata/real.csv']);

% 提取座標
pxx = real_data.px;
pyy = real_data.py;
x_back = back_data.x;
y_back = back_data.y;
px_sim = sim_data.px;
py_sim = sim_data.py;

%% 2. 計算 Real Data 誤差 (帶正負號)
error = zeros(length(pxx), 1);
for i = 1:length(pxx)
    px = pxx(i); py = pyy(i);
    min_dist = inf;
    side_indicator = 0;
    
    for j = 1:(length(x_back)-1)
        x1 = x_back(j);   y1 = y_back(j);
        x2 = x_back(j+1); y2 = y_back(j+1);
        
        v = [x2-x1, y2-y1]; % 線段向量
        w = [px-x1, py-y1]; % 點到起點向量
        
        % 計算投影比例 t
        t = dot(w,v) / dot(v,v);
        
        if t < 0
            dist = sqrt((px-x1)^2 + (py-y1)^2);
        elseif t > 1
            dist = sqrt((px-x2)^2 + (py-y2)^2);
        else
            proj = [x1, y1] + t*v;
            dist = sqrt((px-proj(1))^2 + (py-proj(2))^2);
        end
        
        % 更新最小距離並紀錄側向方位
        if dist < min_dist
            min_dist = dist;
            % 2D 外積公式: (x2-x1)*(py-y1) - (y2-y1)*(px-x1)
            % > 0 在左側, < 0 在右側
            side_indicator = (x2 - x1) * (py - y1) - (y2 - y1) * (px - x1);
        end
    end
    error(i) = sign(side_indicator) * min_dist;
end

%% 3. 計算 Simulation 誤差 (帶正負號)
error_sim = zeros(length(px_sim), 1);
for i = 1:length(px_sim)
    px = px_sim(i); py = py_sim(i);
    min_dist = inf;
    side_indicator = 0;
    
    for j = 1:(length(x_back)-1)
        x1 = x_back(j);   y1 = y_back(j);
        x2 = x_back(j+1); y2 = y_back(j+1);
        v = [x2-x1, y2-y1]; w = [px-x1, py-y1];
        t = dot(w,v) / dot(v,v);
        
        if t < 0
            dist = sqrt((px-x1)^2 + (py-y1)^2);
        elseif t > 1
            dist = sqrt((px-x2)^2 + (py-y2)^2);
        else
            proj = [x1, y1] + t*v;
            dist = sqrt((px-proj(1))^2 + (py-proj(2))^2);
        end
        
        if dist < min_dist
            min_dist = dist;
            side_indicator = (x2 - x1) * (py - y1) - (y2 - y1) * (px - x1);
        end
    end
    error_sim(i) = sign(side_indicator) * min_dist;
end

%% 4. 統計計算
% 距離誤差統計
rmse = sqrt(mean(error.^2));
max_error = max(abs(error));       % 改用絕對值找最大偏差
mean_error = mean(error);          % 平均值可看出是否有系統性偏置

rmse_sim = sqrt(mean(error_sim.^2));
max_error_sim = max(abs(error_sim));
mean_error_sim = mean(error_sim);

% epsi 統計 (航向角誤差)
epsi_deg = rad2deg(real_data.epsi);
rmse_eps_deg  = sqrt(mean(epsi_deg.^2));
max_eps_deg   = max(abs(epsi_deg));

%% 5. 繪圖
% --- 圖一：路徑比較 ---
figure('Color', 'w', 'Name', 'Trajectory Comparison');
plot(x_back, y_back, 'k--', 'LineWidth', 2); hold on; % 參考路徑改黑色虛線較清楚
plot(pxx, pyy, 'b-', 'LineWidth', 1.5);             % 真實路徑
plot(px_sim, py_sim, 'r-.', 'LineWidth', 1.5);      % 模擬路徑
legend('Reference Path', 'Real Trajectory', 'Simulation Trajectory', 'Location', 'best');
xlabel('X (m)'); ylabel('Y (m)');
title('Path Comparison: Real vs Simulation vs Reference');
grid on; axis equal;

% --- 圖二：帶正負號的誤差曲線 ---
figure('Color', 'w', 'Name', 'Error Analysis');
plot(error, 'b-', 'LineWidth', 1.2); hold on;
plot(error_sim, 'r-', 'LineWidth', 1.2);
yline(0, 'k--', 'LineWidth', 1); % 加入 0 準位線
ylim([-2, 2]);
xlabel('Data Index'); ylabel('Cross-Track Error (m)');
legend(['Real (Max Abs:' num2str(max_error,'%.3f') ')'], ...
       ['Sim (Max Abs:' num2str(max_error_sim,'%.3f') ')'], ...
       'Center Line');
title(['Signed Error Analysis | Real RMSE: ' num2str(rmse,'%.4f') ...
       ' | Sim RMSE: ' num2str(rmse_sim,'%.4f')]);
grid on;

% 輸出簡單報告到終端機
fprintf('--- 統計結果 ---\n');
fprintf('Real Data -> RMSE: %.4f m, Max Error: %.4f m, Mean Error: %.4f m\n', rmse, max_error, mean_error);
fprintf('Sim  Data -> RMSE: %.4f m, Max Error: %.4f m, Mean Error: %.4f m\n', rmse_sim, max_error_sim, mean_error_sim);