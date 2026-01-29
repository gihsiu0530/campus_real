% 讀取資料
real_data = readtable('/home/cyc/new_golf/src/mpc/mpcdata/simulation.csv');
back_data = readtable('/home/cyc/back_points.csv');

% 取出 v_real < 0 的索引
neg_idx = real_data.v_real < 0;

% 僅保留負速度部分
pxx  = real_data.px(neg_idx);
pyy  = real_data.py(neg_idx);
epsi = real_data.epsi(neg_idx);
epsi_deg = rad2deg(epsi);

x_back = back_data.x;
y_back = back_data.y;
error = zeros(length(pxx),1);

% 計算距離誤差
for i = 1:length(pxx)
    px = pxx(i);
    py = pyy(i);
    min_dist = inf;
    for j = 1:(length(x_back)-1)
        x1 = x_back(j);   y1 = y_back(j);
        x2 = x_back(j+1); y2 = y_back(j+1);
        v = [x2-x1, y2-y1];
        w = [px-x1, py-y1];
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
        end
    end
    error(i) = min_dist;
end

% 統計結果
rmse = sqrt(mean(error.^2));
mean_error = mean(error);
max_error = max(error);

% epsi 統計（度）
rmse_eps_deg  = sqrt(mean(epsi_deg.^2));
mean_eps_deg  = mean(epsi_deg);
max_eps_deg   = max(abs(epsi_deg));

% ===== 繪圖 =====
figure;
plot(pxx, pyy, 'b-', 'LineWidth', 1.5); hold on;
plot(x_back, y_back, 'r--', 'LineWidth', 1.5);
legend('v<0 Real Path', 'Back Path');
xlabel('X'); ylabel('Y');
title('倒車區段路徑比較');
grid on;

figure;
plot(error, 'k-', 'LineWidth', 1.5);
xlabel('Index'); ylabel('距離誤差');
title(['v<0 誤差曲線 | RMSE=' num2str(rmse,'%.4f') ...
       ', Mean=' num2str(mean_error,'%.4f') ...
       ', Max=' num2str(max_error,'%.4f') ...
       ', epsi RMSE=' num2str(rmse_eps_deg,'%.3f') '°' ...
       ', Mean=' num2str(mean_eps_deg,'%.3f') '°' ...
       ', |epsi| Max=' num2str(max_eps_deg,'%.3f') '°']);
grid on;
