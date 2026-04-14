% 讀取資料
real_data = readtable(['/home/cyc/campus_ws/mpcdata/real.csv']);
back_data = readtable('/home/cyc/campus_ws/src/mpc_4state/src/0414.csv');
pxx = real_data.px;
pyy = real_data.py;
x_back = back_data.x;
y_back = back_data.y;
error = zeros(length(pxx),1);
epsi = real_data.epsi;
epsi_deg = rad2deg(epsi);
% 計算每個 (pxx,pyy) 到路徑 polyline 的最短距離
for i = 1:length(pxx)
    px = pxx(i);
    py = pyy(i);
    min_dist = inf;
    for j = 1:(length(x_back)-1)
        % 線段端點
        x1 = x_back(j);   y1 = y_back(j);
        x2 = x_back(j+1); y2 = y_back(j+1);
        % 線段向量與點向量
        v = [x2-x1, y2-y1];
        w = [px-x1, py-y1];
        % 投影比例 t
        t = dot(w,v) / dot(v,v);
        if t < 0
            % 最近點是端點 (x1,y1)
            dist = sqrt((px-x1)^2 + (py-y1)^2);
        elseif t > 1
            % 最近點是端點 (x2,y2)
            dist = sqrt((px-x2)^2 + (py-y2)^2);
        else
            % 最近點在線段上
            proj = [x1, y1] + t*v;
            dist = sqrt((px-proj(1))^2 + (py-proj(2))^2);
        end
        % 更新最小距離
        if dist < min_dist
            min_dist = dist;
        end
    end
    error(i) = min_dist;
end

rmse = sqrt(mean(error.^2));
mean_error = mean(error);
max_error = max(error);

% epsi 統計（度）
rmse_eps_deg  = sqrt(mean(epsi_deg.^2));
mean_eps_deg  = mean(epsi_deg);
max_eps_deg   = max(abs(epsi_deg));   % 最大絕對角誤差（度）
% ===========================

% ===== 圖一：路徑比較 =====
figure;
plot(pxx, pyy, 'b-', 'LineWidth', 1.5); hold on;
plot(x_back, y_back, 'r--', 'LineWidth', 1.5);
legend('Real (pxx,pyy)', 'Back Path (x,y)');
xlabel('X'); ylabel('Y');
title('路徑比較');
grid on;

% ===== 圖二：誤差曲線 =====
figure;
plot(error, 'k-', 'LineWidth', 1.5);
xlabel('Index'); ylabel('最短距離誤差');
% ===== 修改：在標題中加入最大誤差 =====
% 使用 ... 來換行，讓標題更清晰
title(['誤差曲線 | Dist RMSE = ' num2str(rmse,'%.4f') ...
       ', Mean = ' num2str(mean_error,'%.4f') ...
       ', Max = ' num2str(max_error,'%.4f') ...
       ', epsi RMSE = ' num2str(rmse_eps_deg,'%.3f') '°' ...
       ', epsi Mean = ' num2str(mean_eps_deg,'%.3f') '°' ...
       ', |epsi| Max = ' num2str(max_eps_deg,'%.3f') '°']);

% ===================================
grid on;