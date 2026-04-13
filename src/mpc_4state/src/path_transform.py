import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

def transform_trajectory(df, dx=0, dy=0, angle_deg=0, center=(0, 0)):
    """
    旋轉並位移軌跡資料。
    
    參數:
    df (pd.DataFrame): 包含 'x' 與 'y' 欄位的資料表。
    dx (float): x 方向位移量。
    dy (float): y 方向位移量。
    angle_deg (float): 旋轉角度（度）。
    center (tuple): 旋轉中心點 (x, y)。
    """
    # 將角度轉為弧度
    angle_rad = np.radians(angle_deg)
    
    # 提取座標
    x = df['x'].values
    y = df['y'].values
    cx, cy = center
    
    # 旋轉公式:
    # x' = (x - cx) * cos(θ) - (y - cy) * sin(θ) + cx
    # y' = (x - cx) * sin(θ) + (y - cy) * cos(θ) + cy
    x_rot = (x - cx) * np.cos(angle_rad) - (y - cy) * np.sin(angle_rad) + cx
    y_rot = (x - cx) * np.sin(angle_rad) + (y - cy) * np.cos(angle_rad) + cy
    
    # 位移
    x_new = x_rot + dx
    y_new = y_rot + dy
    
    return pd.DataFrame({'x': x_new, 'y': y_new})

# 1. 讀取原始資料
df_original = pd.read_csv('/home/cyc/campus_ws/src/mpc_4state/src/transformed_points1.csv')

# 2. 設定變換參數 (例如: 以第一點為中心旋轉 45 度，並平移 x+10, y+10)
start_point = (df_original['x'].iloc[0], df_original['y'].iloc[0])
df_transformed = transform_trajectory(
    df_original, 
    dx=-1,   #-90
    dy=0,    #55 
    angle_deg=0, #170 
    center=start_point
)

# 3. 儲存結果
df_transformed.to_csv('transformed_points.csv', index=False)

# # 4. 繪製圖表展示
# plt.figure(figsize=(10, 6))
# plt.plot(df_original['x'], df_original['y'], 'b--', label='Original', alpha=0.5)
# plt.plot(df_transformed['x'], df_transformed['y'], 'r-', label='Transformed (45°, dx=10, dy=10)')
# plt.legend()
# plt.grid(True)
# plt.axis('equal')
# plt.show()
