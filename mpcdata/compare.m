% 讀取資料
real_data = readtable(['/home/cyc/new_golf/src/mpc/mpcdata/real_0923_4.csv']);
path_data = readtable('/home/cyc/back_points.csv');
path1_data = readtable('/home/cyc/path_points.csv');

% 鏡射中心
xc = 17.4885;
yc = 24.9375;

% 初始化新的座標
px_new = real_data.px;
py_new = real_data.py;

% 找出需要鏡射的點
idx = real_data.px > 17.1549;

% 對符合條件的點做鏡射
px_new(idx) = 2*xc - real_data.px(idx);
py_new(idx) = 2*yc - real_data.py(idx);

% 繪製 px, py 與 path_points 的比較
figure;
plot(real_data.px, real_data.py, 'g-', 'LineWidth', 1.5); hold on;
plot(real_data.pxx, real_data.pyy, 'b-', 'LineWidth', 1.5); hold on;
plot(path_data.x, path_data.y, 'r--', 'LineWidth', 1.5);
plot(path1_data.x, path1_data.y, 'k--', 'LineWidth', 1.5);

xlabel('X 座標');
ylabel('Y 座標');
legend('Real Trajectory (px, py)', 'Path Points (x, y)');
title('路徑比較');
grid on;
xlim([0 50]);
ylim([0 50]);

% 誤差計算
error_curve = real_data.v_ref - abs(real_data.v_real);

% 平均速度誤差
mean_error = mean(error_curve);

% 顯示結果
disp(['平均速度誤差 = ', num2str(mean_error)]);


% 繪製 v_ref 與 v_real 比較
figure;
plot(real_data.v_ref, 'r-', 'LineWidth', 1.5); hold on;
plot(abs(real_data.v_real), 'b-', 'LineWidth', 1.5);
% --- 新增: 將 u_a 積分，當作速度曲線 ---
dt = 0.1;  % 假設每筆資料的時間間隔是 0.1 秒 (依你的資料實際情況修改)
u_a_integral = cumsum(real_data.u_a) * dt;
%plot(u_a_integral, 'g-', 'LineWidth', 1.5);
xlabel('時間索引');
ylabel('速度');
legend('v\_ref', 'v\_real');
title('v\_ref 與 v\_real 比較');
grid on;

% 繪製 v_ref - v_real 的誤差曲線
ylim([-0.8 0.8]);
figure;
error_curve = real_data.v_ref - abs(real_data.v_real);
plot(error_curve, 'k-', 'LineWidth', 1.5);
xlabel('時間索引');
ylabel('誤差 (v\_ref - v\_real)');
title('速度誤差曲線');
grid on;
