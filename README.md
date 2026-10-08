# campus_ws — 視覺路徑推論 + MPC 自駕車堆疊

校園高爾夫球車的自駕堆疊：用相機影像**推論前方路徑**，交給 **MPC** 追蹤，
輸出方向盤角度與目標車速。定位可以用 **VIO（OpenVINS，純視覺）** 或 **光達（hdl_localization）**。

- [1. 系統架構](#1-系統架構)
- [2. 要開哪些程式](#2-要開哪些程式)
- [3. 啟動步驟](#3-啟動步驟)
- [4. 各元件怎麼運作](#4-各元件怎麼運作)
- [5. 怎麼串接控制程式](#5-怎麼串接控制程式)
- [6. 純軟體模擬](#6-純軟體模擬)
- [7. 監看與除錯](#7-監看與除錯)
- [8. 已知限制](#8-已知限制)

---

## 1. 系統架構

兩種執行模式，由 MPC 的兩個參數決定：

| 模式 | `path_source` | `localization_source` | 參考路徑 | 定位 |
|---|---|---|---|---|
| **視覺推論 + VIO**（本文件主要內容） | `planner` | `vio` | 路徑推論程式每 0.5 s 發一條約 4 m 的路徑 | OpenVINS |
| 視覺推論 + 光達 | `planner` | `lidar` | 同上 | hdl_localization `/odom` |
| 固定路線（已驗證 baseline） | `global` | `lidar` | CSV 路線（`global_path`） | hdl_localization `/odom` |

視覺推論 + VIO 模式的資料流：

```
ZED2i ──左右影像 + IMU──▶ OpenVINS ── /ov_msckf/poseimu ──▶ vio_pose_to_odom.py ── /vio_odom ─┐
  │                                                                                           │
  └──左影像──▶ 路徑推論程式 (realtime_planner_node_ff_VIO.py) ◀── 位姿快照 ────────────────────┤
                 │  語意分割 → 深度 → 模型推論 → 平滑 → 錨定到 VIO 座標                          │
                 └── /senpai/array_topic ──▶ MPC (mpc_back_test) ◀── 車輛位姿 ───────────────┘
                                               │  ◀── /v_real、/steering_sensor
                                               ├── /mpc_result_test [u_a, u_delta, v_ref]
                                               │      ├─▶ steering_test/control_mpc ─▶ 方向盤
                                               │      └─▶ Arduino (rosserial) ───────▶ 油門 PI
                                               └── /stop_signal ──▶ Arduino（停車）
```

重點：**推論路徑和 MPC 用同一個 `/vio_odom`**，兩者才在同一個座標系（`global`）。

---

## 2. 要開哪些程式

| # | 程式 | 位置 | 作用 | 由誰啟動 |
|---|---|---|---|---|
| 1 | roscore、Velodyne、**ZED2i driver**、hdl_localization、方向盤控制、`/v_real` 發佈 | `~/golf_ws/start.sh` | 感測器與車輛底層 | `start.sh` |
| 2 | **OpenVINS** | `~/open_vins_P`（`subscribe.launch`） | VIO 定位 | `soturon.sh`（vio 格） |
| 3 | **vio_pose_to_odom.py** | `src/mpcbitch/scripts/` | 位姿轉型別 | `soturon.sh`（vio2odom 格） |
| 4 | **路徑推論程式** | `path_inference/real_time_ff/realtime_planner_node_ff_VIO.py` | 產生參考路徑 | `soturon.sh`（planner 格） |
| 5 | keyboard_command.py | `path_inference/real_time_ff/` | 路口指令（左/直/右） | `soturon.sh`（keyboard 格） |
| 6 | rosserial | `rosserial_python serial_node.py` | 與 Arduino（`~/Arduino/weirdo/weirdo.ino`）通訊，控制油門 | `soturon.sh`（rosserial 格） |
| 7 | rviz | `src/mpcbitch/rviz/vio_planner.rviz` | 監看 | `soturon.sh`（rviz 格） |
| 8 | **MPC 控制程式** | `src/mpcbitch/src/mpc_back_test.cpp` | 追蹤路徑、輸出轉向/車速 | `soturon.sh`（mpc 格，**需手動按下**） |

`start.sh` 內各分頁（依序）：Velodyne、ZED2i（`zed_wrapper zed2i.launch`）、hdl_localization（地圖參數）、
`golf_steering_control`、`steering_test control_mpc`、`mpc_4state global_path`、以及 `~/new_golf` 的
`mpcbitch mpc`（節點名 `mpc_and_vreal_publisher`，發佈 `/v_real`）。

---

## 3. 啟動步驟

```bash
# 1. 底層：感測器、方向盤、車速（地圖名稱可省略，預設 garden_fix）
~/golf_ws/start.sh [map_name]

# 2. 自駕堆疊：開一個 terminator 視窗（7 格）
cd ~/campus_ws && ./soturon.sh
```

`soturon.sh` 的版面（每格都能 Ctrl-C 停止、↑ + Enter 重跑）：

```
+----------------------+----------------------+
| vio       (自動)      | vio2odom  (自動)      |
|                      +----------------------+
|                      | rviz      (自動)      |
+----------------------+----------------------+
| planner   (自動)      | rosserial (自動)      |
+----------------------+----------------------+
| keyboard  (自動)      | mpc       (手動)      |
+----------------------+----------------------+
```

3. **車子保持靜止約 2 秒**，等 OpenVINS 初始化。確認 VIO 正常：
   ```bash
   rostopic echo /ov_msckf/poseimu/pose/pose/position   # 靜止時應在原點附近、數值穩定
   ```
4. 等 planner 格開始輸出 `model inference=... ms`，rviz 看到橘色推論路徑。
5. 在 **mpc 格按 ↑ + Enter** 啟動 MPC：
   ```bash
   roslaunch mpcbitch run_mpc.launch path_source:=planner localization_source:=vio
   ```
   log 應出現 `MPC tracking source = planner` 與 `localization source = vio (/vio_odom)`。

`bash soturon.sh --dry-run` 只產生設定檔，不會啟動任何東西。

---

## 4. 各元件怎麼運作

### 4.1 OpenVINS（VIO）

- 實際執行的是 **`~/open_vins_P`**（已編譯好的那份）；`~/open_vins` 是另一份，不要用。
- 輸入：`/zed2i/zed_node/left|right/image_rect_color`、`/zed2i/zed_node/imu/data_raw`（config `zed2i`）。
- 輸出：`/ov_msckf/poseimu`（`PoseWithCovarianceStamped`，frame `global`）、`/ov_msckf/pathimu`、
  `/ov_msckf/trackhist`，以及 TF `global → imu`。
- `dosave:=true` 會把估計軌跡寫到 `~/open_vins_P/vio_estimate_zed2i_builtin_imu.csv`，
  **每次啟動都會覆寫**；需要保留（例如發散紀錄）請另存，例如 `~/open_vins_P/vio_logs/`。
- 原點與 yaw 取決於初始化那一刻，和光達地圖座標無關。

### 4.2 vio_pose_to_odom.py（轉換節點）

- 把 `/ov_msckf/poseimu`（`PoseWithCovarianceStamped`）轉成 `/vio_odom`（`nav_msgs/Odometry`），
  因為推論程式與 MPC 都只吃 `Odometry`。
- 時間戳、frame、位置、姿態、共變異數**原封不動**；twist 填 0（車速另由 `/v_real` 提供）。
- 參數：`~input_topic`、`~output_topic`、`~child_frame_id`、`~lever_arm_x/y/z`（IMU → 車輛參考點，
  預設 0；光達裝在相機正上方，所以與光達模式的參考點水平一致）。

### 4.3 路徑推論程式（`realtime_planner_node_ff_VIO.py`）

**必須用 `stp3_ros` conda 環境**（系統 python 缺套件）。每 0.5 s 處理一張影像：

1. **位姿快照**：影像進來時取一筆 `/vio_odom`，這次推論的 ego 歷史、路徑錨點、繪圖全用它
   （`/odom` 會被忽略）。
2. **語意分割**（`_segmentation_backend`：`segformer` / `yolo26l`）→ 224×224 四類（道路/行人/可移動物/靜態）。
   - **分割防呆**：這幀道路區域與上一幀「採用的」分割差異（1 − IoU）≥ `_seg_gate_max_diff`（0.85）時，
     沿用上一幀；最多連續沿用 `_seg_gate_max_hold`（3）幀，之後強制採用當下結果。
3. **深度**（Depth-Anything-V2，`_use_depth:=true`）→ **模型推論**出 6 個未來點（每 0.5 s 一點）。
4. **路徑後處理**：B-spline 平滑並以 0.7 m 等距重取樣（`_path_smoothing`、`_path_point_spacing_m`），
   再在起點後方沿起始方向**補一個共線點**（`_path_back_extension_m`，預設 0.7 m）。
   這一點是給 MPC 用的：MPC 追蹤點在車輛後方 0.5 m，低速時會落在路徑起點之前，沒有這點 cte 會假性卡在約 0.5 m。
5. 用位姿快照把路徑轉到 `global` 座標後發佈。

| 輸出 topic | 型別 | 內容 |
|---|---|---|
| `/senpai/array_topic` | `Float64MultiArray` | 給 MPC，`[x0,y0,x1,y1,...]`，8 點，`global` 座標 |
| `/senpai/path_global` | `nav_msgs/Path` | 同一批點，給 rviz |
| `/senpai/path` | `nav_msgs/Path` | 車體座標（`base_link`） |
| `/senpai/seg_cls4_224` | `Image` | 實際採用的分割結果 |

- 路口指令來自 `/senpai/command`（`keyboard_command.py`：← / a = LEFT、↑ / w / 空白 = FORWARD、→ / d = RIGHT）。
- `_ego_input_mode:=fixed_speed _fixed_speed_mps:=1.0` 時，餵給模型的過去運動是合成的直線 1 m/s；
  `real_odom` 則用 VIO 實際運動。兩種模式路徑錨點都用 VIO。
- 常用完整指令與效果註記在 `path_inference/real_time_ff/command.txt`。

### 4.4 MPC 控制程式（`mpc_back_test`）

- 以 OSQP 解 MPC，10 Hz，由位姿訊息觸發。
- `~path_source`：`planner` 訂 `/senpai/array_topic`；`global` 訂 `array_topic`（CSV 路線，已驗證 baseline）。
- `~localization_source`：`vio` 訂 `/vio_odom`；`lidar` 訂 `/odom`。**`vio` 只能配 `planner`**，
  global 模式設 vio 會 ROS_ERROR 並強制改回 lidar（CSV 路線是光達地圖座標）。
- `~PLAN_TIMEOUT`（1.5 s）：planner 模式下超過這個時間沒收到新路徑就送停車（`v_ref = 0`，方向盤維持）。
- 追蹤點：位姿往後 0.5 m（`offset = -0.5`），再依 `~state_projection_delay`（預設 0.5 s）往前推算補償延遲。
- 車速取自 `/v_real`，方向盤實際角度取自 `/steering_sensor`。

---

## 5. 怎麼串接控制程式

### 5.1 MPC 的輸入

| Topic | 型別 | 來源 | 用途 |
|---|---|---|---|
| `/senpai/array_topic` | `std_msgs/Float64MultiArray`，`[x0,y0,x1,y1,...]` | 路徑推論程式 | 參考路徑（planner 模式） |
| `/vio_odom` | `nav_msgs/Odometry` | vio_pose_to_odom.py | 車輛位姿（x, y, yaw） |
| `/v_real` | `std_msgs/Float64` | `start.sh` 的 `mpc_and_vreal_publisher` | 實際車速 |
| `/steering_sensor` | `std_msgs/Float32` | 方向盤角度感測（`~/golf_ws/control/steer/`） | 實際方向盤角度 |
| `/max_v`、`/max_v_inc` | `std_msgs/Float64` | 選用 | 執行中調整速度上限 |

**要讓其他路徑來源接上 MPC**：發佈同樣格式的 `Float64MultiArray` 到 `/senpai/array_topic`，
座標必須和 MPC 使用的位姿**在同一個座標系**；點距建議 0.7 m，並在起點後方保留一點（見 4.3 第 4 步）。

### 5.2 MPC 的輸出

| Topic | 型別 | 內容 | 接收端 |
|---|---|---|---|
| `/mpc_result_test` | `std_msgs/Float64MultiArray` | `[u_a 加速度, u_delta 前輪角 (rad), v_ref 目標車速 (m/s)]` | `steering_test/control_mpc` 取 `data[1]` → `/cmd_to_steering` → `golf_steering_control` 轉方向盤；Arduino 取 `data[2]` 做速度 PI 控油門 |
| `/stop_signal` | `std_msgs/Bool` | `true` = 停車 | Arduino |
| `/start_id` | `std_msgs/Int32` | 目前最近的路徑點索引 | 除錯 |

Arduino（`~/Arduino/weirdo/weirdo.ino`）同時訂閱 `/v_real` 當速度回授、`/trans_signal` 切換前進/倒退。

### 5.3 重要參數（`src/mpcbitch/launch/run_mpc.launch`）

| 參數 | 說明 |
|---|---|
| `path_source`（launch arg） | `planner` / `global`，launch 預設 `global` |
| `localization_source`（launch arg） | `vio` / `lidar`，預設 `lidar` |
| `PLAN_TIMEOUT` | planner 路徑逾時停車秒數 |
| `min_v_forward`、`max_v_forward` | 前進速度上下限（m/s） |
| `max_delta_inc` | 每個控制週期最大轉角變化（rad） |
| `R_steer` | 方向盤變化權重 |

參數要寫在 `<node>` 內（程式從私有命名空間讀）。編譯：`cd ~/campus_ws && catkin_make && source devel/setup.bash`。

---

## 6. 純軟體模擬

`run_mpc_sim.launch` 不需要任何感測器或車輛硬體。`mpc_simulate` 當車輛模型
（自行車模型，軸距 1.66 m、步長 0.1 s）：吃 MPC 的控制輸出、積分出位姿再餵回 MPC，
形成閉環。用來在上實車前驗證控制程式的改動。

> **模擬跑的是 `mpc`（`src/mpcbitch/src/mpc.cpp`），不是實車的
> `mpc_back_test`（`src/mpcbitch/src/mpc_back_test.cpp`）。**
> 兩支是平行維護的分支，演算法相同，改完一邊要手動同步到另一邊
> （git log 裡的 `Port the sim MPC improvements to the real-vehicle node`）。

### 6.1 跑起來

```bash
cd ~/campus_ws && catkin_make && source devel/setup.bash

roslaunch mpcbitch run_mpc_sim.launch                       # CSV 路線，跑完自動結束
roslaunch mpcbitch run_mpc_sim.launch rviz:=true            # 同時開 rviz
roslaunch mpcbitch run_mpc_sim.launch path_source:=planner  # 改用 fake_planner 的滾動短路徑
roslaunch mpcbitch run_mpc_sim.launch loop_route:=true      # 當閉環一直繞圈（Ctrl-C 結束）
roslaunch mpcbitch run_mpc_sim.launch path_file:=/abs/route.csv   # 換路線
```

rviz 由 `rviz_display.launch` 帶起（`rviz:=true` 時才 include），載入
`src/mpcbitch/rviz/display.rviz`；也可以單獨開：
`roslaunch mpcbitch rviz_display.launch`，或用 `rviz_config:=` 指定別的設定檔。

### 6.2 節點組成

| 節點 | 程式 | 作用 |
|---|---|---|
| `global_path` | `src/mpc_4state/src/global_path.cpp` | 每秒把整條 CSV 路線發到 `array_topic` |
| `mpc_simulate` | `src/mpc_4state/src/mpc_simulate.cpp` | 車輛模型。訂 `/mpc_result_test`，發 `/mpc_new_pose`、`/v_real`、`/steering_sensor`、車輛 marker。**`required="true"`**：它一結束，roslaunch 就把所有節點關掉 |
| `mpc` | `src/mpcbitch/src/mpc.cpp` | 待驗證的 MPC（10 Hz，OSQP） |
| `fake_planner` | `src/mpcbitch/scripts/fake_planner.py` | 只在 `path_source:=planner` 時啟動。從 CSV 路線切出約 3 s 的滾動短路徑發 `/senpai/array_topic`，模仿 `realtime_planner_node_ff_VIO.py` 的輸出格式 |
| `cte_plotter_node` | `src/mpcbitch/scripts/plot_cte.py` | 關閉時讀 `sim_dir` 裡最新的 csv，畫出 `<同名>_cte.png` |
| `rviz` | `launch/rviz_display.launch` | 只在 `rviz:=true` 時啟動 |

### 6.3 輸出

| 檔案 | 內容 |
|---|---|
| `mpcdata/simulation/sim_YYYYmmdd_HHMMSS.csv` | 逐控制週期紀錄：`u_a, v_real, v_ref, u_delta, delta_d, px, py, theta1, cte, cte_real, epsi, vx, vy, kappa, beta` |
| `mpcdata/simulation/sim_YYYYmmdd_HHMMSS_cte.png` | 關閉時由 `plot_cte.py` 自動產生 |

每次執行都會新開一個檔名帶時間戳的 csv，不會覆寫上一次。

### 6.4 怎麼結束

| 情況 | 結束方式 |
|---|---|
| 一般（`loop_route:=false`） | 車走到路線末端，`endpoint_phase` 把 `v_ref` 降到 0；`mpc_simulate` 偵測到靜止超過 `auto_stop_duration` 就自己結束，`required="true"` 讓 roslaunch 連帶全關，`plot_cte.py` 也就把圖寫出來 |
| 繞圈（`loop_route:=true`） | 車永遠不停，`auto_stop` 不會觸發 → **按 Ctrl-C**。一樣會觸發 roslaunch 關閉流程，圖照樣產生 |

### 6.5 launch arg（命令行可覆寫）

| arg | 預設 | 意義 |
|---|---|---|
| `rviz` | `false` | `true` 時 include `rviz_display.launch` |
| `sim_dir` | `mpcdata/simulation` | 紀錄 csv 與 cte 圖的輸出目錄 |
| `path_source` | `global` | `global` = 跟 `global_path` 發的整條 CSV 路線；`planner` = 跟 `fake_planner.py` 的滾動短路徑（同時會啟動該節點） |
| `loop_route` | `false` | `true` = 把 CSV 路線當閉環，走到最後一點接回第一點繼續繞，不停車也不關節點。需要 `path_source:=global` |
| `path_file` | `path/smoothed/back_garden_07new.csv` | `global_path` 要讀的路線 CSV（給絕對路徑） |
| `planner_ego_mode` | `fixed_speed` | `fake_planner` 餵給自己的 ego 速度來源：`fixed_speed` 用固定值、`real_odom` 用模擬器實際車速 |
| `planner_fixed_speed` | `1.0` | 前者的固定速度（m/s）。短路徑長度約等於 速度 × 3 s |

### 6.6 `mpc_simulate` 的參數（車輛模型）

| param | launch 值 | 程式預設 | 意義 |
|---|---|---|---|
| `emulate_real_sensors` | `true` | `false` | 額外發佈 `/v_real` 與 `/steering_sensor`，並把位姿放在光達安裝點，讓 MPC 收到的訊號與實車一致 |
| `auto_stop` | `true` | `false` | 車靜止夠久就結束這次 run |
| `auto_stop_speed` | `0.09` | `0.02` | 視為「靜止」的速度門檻（m/s） |
| `auto_stop_duration` | `3.0` | `5.0` | 需持續靜止幾秒才結束 |

launch 沒設的 `lidar_offset`（程式預設 0.5 m）是光達安裝點在後軸前方的距離：模擬器把位姿報在該點，
MPC 端再用內部的 `offset = -0.5` 推回車輛參考點，和實車的訊號鏈一致。
`emulate_real_sensors` 為 `false` 時這個偏移會被歸零。

### 6.7 `mpc` 的參數

**路徑與定位**

| param | launch 值 | 程式預設 | 意義 |
|---|---|---|---|
| `path_source` | `$(arg path_source)` | `global` | 參考路徑來源 |
| `loop_route` | `$(arg loop_route)` | `false` | 閉環繞圈。啟動時會檢查路線首尾行進方向是否連續，並自動修剪末端「越過起點又折回」的點 |
| `lidar_odom_topic` | `/mpc_new_pose` | `/odom` | 位姿來源。模擬器發在 `/mpc_new_pose`，所以必須改 |
| `save_dir` | `$(arg sim_dir)` | 未設 | 設了就把紀錄寫成 `<save_dir>/sim_<時間戳>.csv`，否則用 `save_filename` |
| `use_state_projection` | `false` | `true` | 是否把位姿往前推算以補償感測延遲。模擬沒有延遲，所以關掉 |

**速度上下限**

| param | launch 值 | 程式預設 | 意義 |
|---|---|---|---|
| `min_v_forward` | `1.0` | `1.0` | 前進最小速度（m/s） |
| `max_v_forward` | `3.0` | `1.2` | 前進最大速度（m/s）。實車 `run_mpc.launch` 設 1.5 |
| `max_delta_inc` | `0.0084` | `0.02` | 每個控制週期最大前輪角變化（rad）。越小轉向越緩 |

**速度剖面（speed profile）** — 只對 `global` 路線生效，planner 路線走下面的 `planner_*`

| param | launch 值 | 程式預設 | 意義 |
|---|---|---|---|
| `speed_profile` | `true` | `false` | 用預先算好的速度剖面（側向加速度上限 + 轉向速率上限，再做前後向傳遞），取代原本的曲率門檻式降速 |
| `speed_profile_a_lat` | `0.4` | `0.6` | 最大側向加速度（m/s²）。launch 註解：`max_v_forward` 3.0 配 0.4、2.5 配 0.6 |
| `speed_profile_a_acc` | `0.15` | `0.15` | 最大縱向加速度（m/s²） |
| `speed_profile_a_dec` | `0.15` | `0.15` | 最大減速度（m/s²），決定入彎前多早開始煞。實車設 0.2 |
| `speed_profile_steer_rate_frac` | `0.5` | `0.5` | 前饋轉向允許用掉 `max_delta_inc` 的比例 |
| `speed_profile_window` | `5` | `5` | 取前後 N 點內最嚴格的限速，避免彎頂速度回升、出彎前又掉下來 |
| `speed_profile_kappa_baseline` | 未設 | `5` | 算曲率時前後各取幾點，越大越平滑 |
| `planner_speed_profile` | `true` | `false` | planner 路線：每收到一條短路徑就重新規劃，末速收在 `min_v_forward`，確保能在看得到的範圍內煞停 |
| `planner_speed_profile_kappa_baseline` | `2` | `2` | 同上。planner 路徑本身已平滑過，基線可以小 |
| `planner_speed_profile_end_speed` | 未設 | `-1`（= `min_v_forward`） | planner 速度剖面的末速 |

**路徑平滑**

| param | launch 值 | 程式預設 | 意義 |
|---|---|---|---|
| `path_smooth_window` | `5` | `0`（關閉） | Savitzky-Golay 平滑路線點（前後 N 點）。去掉 CSV 約 1 cm 的雜訊，點位移動 ≤ 約 4 cm。只對 global 路線生效 |

**前饋轉向**

| param | launch 值 | 程式預設 | 意義 |
|---|---|---|---|
| `ff_lookahead_time` | `0` | `0.0` | 前視距離 = max(`ff_lookahead_min`, 車速 × 此值)。0 = 用固定前視 |
| `ff_lookahead_min` | `1.87` | `1.87` | 前視距離下限（m） |
| `clean_feedforward` | `false` | `false` | 改用速度剖面算出的乾淨曲率做前饋轉向，取代逐點曲率（逐點曲率的雜訊會讓 `delta_d` 跳動） |
| `clean_feedforward_lookahead` | `2.67` | `2.67` | 前者的前視距離（m） |

**轉向平順度**

| param | launch 值 | 程式預設 | 意義 |
|---|---|---|---|
| `R_steer` | `100` | `30` | MPC 對方向盤變化量的權重。越大越平穩但反應越慢 |
| `CTE_ENTER` | `0.0` | `0.015` | 轉向死區的進入門檻（m）。設 0 時進入條件永不成立，等於關閉死區 |

### 6.8 `fake_planner.py` 的參數

只在 `path_source:=planner` 時啟動。launch 只設了前兩個，其餘用程式預設。

| param | launch 值 | 程式預設 | 意義 |
|---|---|---|---|
| `ego_input_mode` | `$(arg planner_ego_mode)` | `fixed_speed` | 決定路徑長度的 ego 速度來源：`fixed_speed` 用固定值、`real_odom` 用模擬器實際車速 |
| `fixed_speed_mps` | `$(arg planner_fixed_speed)` | `1.0` | 前者的固定速度（m/s） |
| `horizon_s` | 未設 | `3.0` | 路徑涵蓋幾秒（長度 ≈ 速度 × 此值） |
| `path_point_spacing_m` | 未設 | `0.7` | 重取樣點距（m），與 CSV 路線一致 |
| `path_min_points` | 未設 | `7` | 最少點數 |
| `path_back_extension_m` | 未設 | 等於點距 | 在起點後方沿起始方向補一個共線點，避免 MPC 的追蹤點（位姿後 0.5 m）落在路徑起點之前（見 4.3 第 4 步） |
| `route_smooth_window` | 未設 | `5` | 切路徑前先平滑 CSV 路線（前後 N 點） |
| `period_s` | 未設 | `0.5` | 重發路徑的週期（s），對應推論程式的約 0.5 s |
| `route_topic` / `out_topic` | 未設 | `array_topic` / `/senpai/array_topic` | 輸入的 CSV 路線、輸出的短路徑 |
| `frame_id` | 未設 | `map` | 發佈路徑的 frame |

### 6.9 `plot_cte.py` 的參數

| param | launch 值 | 程式預設 | 意義 |
|---|---|---|---|
| `csv_dir` | `$(arg sim_dir)` | `''` | 關閉時從這個目錄找最新的 csv 畫圖；留空則改用 `csv_file` / `img_file` |

### 6.10 模擬與實車的差異

| 項目 | 模擬 | 實車 |
|---|---|---|
| 執行檔 | `mpc`（`src/mpc.cpp`） | `mpc_back_test`（`src/mpc_back_test.cpp`） |
| launch | `run_mpc_sim.launch` | `run_mpc.launch` |
| 位姿來源 | `/mpc_new_pose`（模型積分出來的） | `/odom`（光達）或 `/vio_odom`（VIO） |
| 感測延遲 | 無，`use_state_projection=false` | 有，預設往前推算 0.5 s |
| 車速 | 模型輸出（`emulate_real_sensors` 模擬成 `/v_real`） | `mpc_and_vreal_publisher` 對光達 `/odom` 微分 |
| 速度上限 | 3.0 m/s | 1.5 m/s |
| `max_delta_inc` | 0.0084 rad | 0.02 rad |

---

## 7. 監看與除錯

- **rviz**（`vio_planner.rviz`，Fixed Frame `global`、視角跟著 `imu`）：
  綠線 = VIO 軌跡、紅箭頭 = `/vio_odom`、橘線 = 推論路徑；下方為 VIO 特徵追蹤影像與分割影像。
  車子靜止時 VIO 軌跡只是一個點。關閉 rviz 若詢問是否儲存，選不儲存以保留預設排列。
- 常用指令：
  ```bash
  rostopic hz /ov_msckf/poseimu /vio_odom /senpai/array_topic /mpc_result_test
  rostopic echo /ov_msckf/poseimu/pose/pose/position
  rostopic echo -n1 /senpai/array_topic
  ```

| 症狀 | 原因與處理 |
|---|---|
| planner 一直印 `waiting for VIO pose /vio_odom` | OpenVINS 或 vio2odom 沒起來 / VIO 尚未初始化 |
| VIO 位置變成數十萬公尺、rviz 路徑報錯 | **VIO 發散**。立刻停車、不要開 MPC，重啟 vio 格並靜止初始化 |
| planner 印 `segmentation gate: holding previous mask` | 分割與上一幀差異過大，正在沿用上一幀；頻繁出現可調高 `_seg_gate_max_diff` |
| MPC 沒有任何輸出 | 沒收到位姿（`/vio_odom`）；MPC 由位姿訊息觸發 |
| MPC log `planner path stale` | 超過 `PLAN_TIMEOUT` 沒收到新路徑，已送停車 |

---

## 8. 已知限制

- **VIO 停止輸出時 MPC 不送任何指令（包含停車）**；VIO 重新初始化或發散時座標跳變不偵測，
  且位姿離路徑超過 5 m 時 MPC 仍會繼續輸出。VIO 模式下請隨時準備人工接管。
- `/v_real` 目前由 `mpc_and_vreal_publisher` 對光達 `/odom` 微分而來，**車速仍依賴光達**。
- OpenVINS 設定 `zupt_only_at_beginning: true`，長時間停車時可能漂移。
- `run_mpc.launch` 的 `MIN_LOOKAHEAD` / `MAX_LOOKAHEAD` / `LOOKAHEAD_GAIN` 沒有接線（刻意保留）。
- `realtime_planner_node_ff_VIO_VLM.py` 是獨立副本，沒有上述 VIO 錨點、補點、分割防呆等改動。
- **沒有任何節點發佈 `/turn_index`**（`campus_ws`、`~/golf_ws`、`~/new_golf` 都查過）。
  因此 `turn_index_` 一直停在初值 0，`turnIndexCallback` 裡寫死的 10000 從未生效。
  後果在模擬與實車都一樣：
  - 最近點搜尋窗口塌成空（`end = 0`），所以 log 裡的 `starting_waypoint_for mpc is: -1`
    是常態，**不代表跟蹤失敗**；`/start_id` 實際發出的恆為 0。
  - 終點判斷 `start_id >= 路徑點數-2` 永不成立 → 那段的 `finish`、`publishStopSignal(true)`、
    `ros::shutdown()` 是**死碼**。車其實是靠終點前 `N_end_slow` 點的速度緩降才停下來的。
  - Frenet 投影（cte / epsi）不受影響：它的邊界有 `i_end <= i_begin` 防呆，會改成掃全路徑，
    所以模擬中 cte 仍維持在約 0.02 m。
  - 倒退狀態機（`APPROACH` / `DWELL` / `RESUME`）同樣進不去。
    若將來讓某個節點發 `/turn_index`，上述路徑會一次全部活過來，需要重新測試。
- `loop_route:=true` 需要**真正的閉環路線**：終點要回到起點附近，而且行進方向連續。
  啟動時程式會檢查，不符就 `ROS_ERROR` 並自動退回單趟模式。
  `path/smoothed/back_garden_07new.csv` 是合格的閉環（首尾方向差 1.7°），
  它末端有 3 個越過起點又折回的點，由程式自動修剪（會印 `trimmed 3 point(s)`）。
