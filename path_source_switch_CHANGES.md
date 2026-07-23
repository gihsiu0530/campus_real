# MPC 參考路徑來源切換（global / planner）改動說明

> 這份文件給「其他裝置上的 Claude」照著重現同一套改動用。
> 目標：在 `mpc_back_test` 節點新增一個參數，讓 MPC 可以在執行期選擇要追
> `global_path.cpp`（CSV 全域路徑）或 `realtime_planner_node.py`（即時推論路徑）。
> 所有程式碼註解、commit message 用英文；本說明用繁體中文。

---

## 1. 目的與背景

原本 `mpc_back_test` 只會訂閱固定 topic `array_topic`，追 `global_path.cpp` 從
CSV 讀出的整條全域路徑。需求是加一個參數，能切換去追
`path_inference/realtime/realtime_planner_node.py` 即時推論產生的路徑。

**關鍵事實**：三者用的是同一種訊息格式，所以切換本質只是「換訂閱的 topic 名稱」：

| 節點 | topic | 型別 | 內容 |
|---|---|---|---|
| `global_path.cpp` | `array_topic` | `std_msgs/Float64MultiArray` | CSV 整條路徑 `[x0,y0,x1,y1,...]` |
| `realtime_planner_node.py` | `array_topic`（**原本預設也叫這個 → 撞名**） | `std_msgs/Float64MultiArray` | 推論出的 7 點軌跡轉 odom 全域座標，格式相同 |
| `mpc_back_test.cpp::setPlan` | 訂閱 `array_topic` | `std_msgs/Float64MultiArray` | 拆成 `global_path_x/y` |

因此改動 = **加一個 `path_source` 參數切換訂閱 topic** + **把 planner 的發布
topic 改名避免與 global_path 撞名**。`setPlan` 的解析邏輯完全不用動。

---

## 2. 需要動到的四個檔案（逐一）

路徑以 workspace 根目錄 `controller_sim` / `senpai` 為基準。

### 2.1 `controller_sim/src/mpcbitch/include/mpcbitch/mpc_final.h`

在 `private:` 區塊、`bool loop_mode_ = false;` 這幾行**之後**，新增三個成員變數
（與 `min_lookahead_`、`loop_mode_` 同一區）：

```cpp
  int look_back_dist_ = 6;
  bool loop_mode_ = false;

  // Reference path source selection
  std::string path_source_ = "global";                  // "global" or "planner"
  std::string global_array_topic_ = "array_topic";      // global_path.cpp source
  std::string planner_array_topic_ = "/senpai/array_topic"; // realtime_planner source
```

> `std::string` 已可用（同檔 `filename_` 就是 `std::string`），不需另加 include。

### 2.2 `controller_sim/src/mpcbitch/src/mpc_back_test.cpp`

**(a) 讀取參數** —— 在 `initialize()` 裡，`LOOP_MODE` 那行參數讀取**之後**、
`setParam` 區塊之前，依現有 `private_nh_.param<T>(...)` 模式新增：

```cpp
  private_nh_.param<int>("LOOK_BACK_DIST", look_back_dist_, 5);
  private_nh_.param<bool>("LOOP_MODE", loop_mode_, false);

  // Reference path source: "global" (global_path.cpp CSV route) or
  // "planner" (realtime_planner_node.py inferred path). Both publish the same
  // std_msgs/Float64MultiArray [x0,y0,x1,y1,...] layout, so switching is just a
  // topic-name change; setPlan stays identical.
  private_nh_.param<std::string>("path_source", path_source_, "global");
  private_nh_.param<std::string>("global_array_topic", global_array_topic_,
                                 "array_topic");
  private_nh_.param<std::string>("planner_array_topic", planner_array_topic_,
                                 "/senpai/array_topic");
```

**(b) 回寫參數（依現有慣例讓 rqt 看得到）** —— 在 `setParam("LOOP_MODE", ...)`
之後新增：

```cpp
  private_nh_.setParam("LOOP_MODE", loop_mode_);

  private_nh_.setParam("path_source", path_source_);
  private_nh_.setParam("global_array_topic", global_array_topic_);
  private_nh_.setParam("planner_array_topic", planner_array_topic_);
```

**(c) 依參數決定訂閱哪個 topic** —— 找到原本的訂閱：

```cpp
    global_path_sub =
        nh_.subscribe("array_topic", 1000, &MPCPlanner_path::setPlan, this);
```

改成：

```cpp
    std::string wp_topic =
        (path_source_ == "planner") ? planner_array_topic_ : global_array_topic_;
    global_path_sub =
        nh_.subscribe(wp_topic, 1000, &MPCPlanner_path::setPlan, this);
    ROS_INFO("MPC tracking source = %s (topic: %s)", path_source_.c_str(),
             wp_topic.c_str());
```

> `setPlan` callback 本身**完全不動**。

### 2.3 `path_inference/realtime/realtime_planner_node.py`

只改 `~array_topic` 的**預設值**，讓它與 global_path 的 `array_topic` 不撞名：

```python
# 改前
self.array_topic = rospy.get_param("~array_topic", "array_topic")
# 改後
self.array_topic = rospy.get_param("~array_topic", "/senpai/array_topic")
```

> 發布格式（`build_array_topic`，`[x0,y0,x1,y1,...]` 的 `Float64MultiArray`）**不動**。

### 2.4 `controller_sim/src/mpcbitch/launch/run_mpc.launch`

在頂層參數區（例如 `LOOP_MODE` 之後）新增 `path_source`。**注意：這個節點的
`private_nh_` 其實是預設建構的全域 handle，所以參數要放在 `<launch>` 頂層
（全域），不要巢狀在 `<node>` 裡**（與 `CTE_ENTER`、`save_filename` 等現有參數
同樣放頂層）：

```xml
    <param name="LOOP_MODE" value="false" />

    <!-- Reference path source for mpc_back_test:
         "global"  -> follow global_path.cpp CSV route on array_topic (default).
         "planner" -> follow realtime_planner_node.py on /senpai/array_topic.
         To use "planner": set this to "planner" AND launch realtime_planner_node.py
         separately (rosrun/python3), since it needs GPU + model checkpoint and is
         not a catkin node. The global_path node below can stay running (it now
         publishes to a different topic than the planner, so no collision). -->
    <param name="path_source" value="global" />
```

---

## 3. 建置

ROS1 (noetic) catkin 工作區。只建這個 target 即可：

```bash
source /opt/ros/noetic/setup.bash
cd <workspace>/controller_sim
catkin_make --source src --build build mpc_back_test
```

預期：編譯通過（只會有一個既有的 `OsqpEigen ... solve() is deprecated` warning，
與本次改動無關）。

---

## 4. 使用與驗證

### 維持原行為（global，預設）
```bash
roslaunch mpcbitch run_mpc.launch
# log 應出現：MPC tracking source = global (topic: array_topic)
```

### 切換到 planner
1. launch 裡把 `path_source` 改成 `planner`。
2. 另外手動啟動 planner（非 catkin 節點，需要 GPU + checkpoint；並把它的
   `~in_topic`(相機影像) 與 `~odom_topic`(里程計) remap 到實際來源，例如 bag 的
   topic 名稱）：
   ```bash
   rosrun ... realtime_planner_node.py \
     _in_topic:=<影像 topic> _odom_topic:=<odom topic>
   ```
3. 驗證：
   ```bash
   # MPC 有沒有切對來源（看 log）：MPC tracking source = planner (topic: /senpai/array_topic)
   rostopic echo /senpai/array_topic   # 應該有 14 個 double 且一直更新
   rostopic info /senpai/array_topic   # 訂閱者應含 mpc_back_test
   ```

---

## 5. 已知行為與注意事項（重要）

- **planner 模式下仍要保留 `global_path` 節點在跑**。因為 `mpc_simulate`
  （`controller_sim/src/mpc_4state/src/mpc_simulate.cpp`）的車輛**初始位姿寫死
  讀 `/array_topic`**（`waitForMessage<Float64MultiArray>("/array_topic")`，取
  `data[2]/data[3]`）。這條**不受 `path_source` 影響**，所以模擬車一律在 CSV
  起點出生 —— 這是預期行為，不是 bug。若把 global_path 關掉，mpc_simulate 會
  收不到初始位姿而卡住。
- RViz 那條紅線是 `global_path` 發的 `MarkerArray`（視覺化），**不是 MPC 追的
  路徑**。要看 planner 的即時路徑，在 RViz 加一個 Path display 訂閱
  `/senpai/path_global`。
- **前視長度差異**：global_path 是整條 CSV（多點）；planner 只有 7 點短程軌跡。
  接 planner 時前視可能偏短，必要時調 `MIN_LOOKAHEAD` / `MAX_LOOKAHEAD` /
  `LOOKAHEAD_GAIN`。
- **座標系**：global_path 用 map/CSV 全域座標；planner 用 odom。確認 MPC 位姿
  來源 `/mpc_new_pose` 與 planner 的 odom 是同一 frame。

---

## 6. 若其他裝置路徑/結構不同：用關鍵字定位

檔案路徑或 launch 內容不一定跟這台一樣。用下列關鍵字 `grep` 到對應位置再改，
不要只認死路徑：

| 要改的地方 | 定位關鍵字（grep） |
|---|---|
| MPC 訂閱參考路徑（要改成依 `path_source_` 選 topic） | `nh_.subscribe("array_topic"` 或 `&MPCPlanner_path::setPlan` |
| MPC 參數讀取區（在此後面加 3 個 `param`） | `LOOP_MODE` 或 `private_nh_.param` |
| MPC 成員變數宣告區（加 3 個 `std::string`） | `bool loop_mode_ = false;` |
| planner 發布 topic 預設名（改 `/senpai/array_topic`） | `~array_topic` 或 `self.array_topic = rospy.get_param` |
| launch 加 `path_source` 參數 | `type="mpc_back_test"` 找到節點所在的 launch，再把 `<param>` 放頂層 |

參考指令：
```bash
grep -rn 'nh_.subscribe("array_topic"' <workspace>          # 找 MPC 訂閱點
grep -rn '~array_topic' <workspace>/senpai                  # 找 planner 發布 topic
grep -rln 'type="mpc_back_test"' <workspace> --include=*.launch  # 找啟動 launch
```

> 注意：同一個 `mpcbitch` 套件裡有很多 `mpc_*.cpp` 變體都有
> `nh_.subscribe("array_topic", ..., &MPCPlanner_path::setPlan, ...)`。這次只需改
> **實際使用的那一支**（本專案是 `mpc_back_test.cpp`）；若其他裝置跑的是別支
> （例如 `mpc_real.cpp`、`mpc_carla.cpp`），就對那一支做同樣改動即可，改法一致。

---

## 7. 快速檢查清單（給重現者）

- [ ] `mpc_final.h`：新增 3 個 `std::string` 成員
- [ ] `mpc_back_test.cpp`：新增 3 個 `param` 讀取
- [ ] `mpc_back_test.cpp`：新增 3 個 `setParam` 回寫
- [ ] `mpc_back_test.cpp`：訂閱改成依 `path_source_` 選 topic + 加 `ROS_INFO`
- [ ] `realtime_planner_node.py`：`~array_topic` 預設改 `/senpai/array_topic`
- [ ] `run_mpc.launch`：頂層加 `<param name="path_source" value="global" />`
- [ ] `catkin_make ... mpc_back_test` 編譯通過
