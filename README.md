# campus_ws — 視覺路徑推論 + MPC 自駕車堆疊

校園高爾夫球車的自駕堆疊：用相機影像**推論前方路徑**，交給 **MPC** 追蹤，
輸出方向盤角度與目標車速。定位可以用 **VIO（OpenVINS，純視覺）** 或 **光達（hdl_localization）**。

- [1. 系統架構](#1-系統架構)
- [2. 要開哪些程式](#2-要開哪些程式)
- [3. 啟動步驟](#3-啟動步驟)
- [4. 各元件怎麼運作](#4-各元件怎麼運作)
- [5. 怎麼串接控制程式](#5-怎麼串接控制程式)
- [6. 監看與除錯](#6-監看與除錯)
- [7. 已知限制](#7-已知限制)

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

## 6. 監看與除錯

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

## 7. 已知限制

- **VIO 停止輸出時 MPC 不送任何指令（包含停車）**；VIO 重新初始化或發散時座標跳變不偵測，
  且位姿離路徑超過 5 m 時 MPC 仍會繼續輸出。VIO 模式下請隨時準備人工接管。
- `/v_real` 目前由 `mpc_and_vreal_publisher` 對光達 `/odom` 微分而來，**車速仍依賴光達**。
- OpenVINS 設定 `zupt_only_at_beginning: true`，長時間停車時可能漂移。
- `run_mpc.launch` 的 `MIN_LOOKAHEAD` / `MAX_LOOKAHEAD` / `LOOKAHEAD_GAIN` 沒有接線（刻意保留）。
- `realtime_planner_node_ff_VIO_VLM.py` 是獨立副本，沒有上述 VIO 錨點、補點、分割防呆等改動。
