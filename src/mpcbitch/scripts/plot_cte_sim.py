#!/usr/bin/env python3
import rospy
import pandas as pd
import matplotlib
matplotlib.use('Agg')  # Background plotting mode to avoid errors in non-GUI environments
import matplotlib.pyplot as plt
import os
import time

def generate_plot():
    rospy.loginfo("[Plotter Node] Interrupt signal (Ctrl+C) received! Waiting for C++ node to shut down and preparing to plot...")
    
    # Wait for 1 second to ensure the C++ node has completely exited and released the file
    time.sleep(1.0)
    
    # The path must be consistent with the C++ code
    csv_file = "/home/cyc/campus_ws/mpcdata/simulation.csv"
    img_file = "/home/cyc/campus_ws/mpcdata/cte_sim.png"
    
    if not os.path.exists(csv_file):
        rospy.logerr(f"[Plotter Node] CSV file not found: {csv_file}. Plotting canceled.")
        return

    try:
        rospy.loginfo("[Plotter Node] Reading data...")
        df = pd.read_csv(csv_file)
        
        if 'cte' not in df.columns:
            rospy.logerr("[Plotter Node] Column 'cte' not found in the CSV file!")
            return
            
        plt.figure(figsize=(10, 5))
        plt.plot(df['cte'], label='CTE (Cross Track Error)', color='b')
        plt.axhline(y=0, color='r', linestyle='--', alpha=0.7)
        plt.title('MPC Control - Cross Track Error')
        plt.xlabel('Time Steps')
        plt.ylabel('Error (m)')
        plt.legend()
        plt.grid(True)
        
        plt.savefig(img_file)
        rospy.loginfo(f"==== Plotting successful! Image saved to: {img_file} ====")
        
    except Exception as e:
        rospy.logerr(f"[Plotter Node] An error occurred during plotting: {e}")

if __name__ == '__main__':
    # Initialize ROS node
    rospy.init_node('cte_plotter_node', disable_signals=False)
    rospy.loginfo("CTE plotter node started. The image will be automatically generated when the program ends or Ctrl+C is pressed.")
    
    # Key mechanism: Register shutdown hook. generate_plot will be called automatically when roslaunch shuts down
    rospy.on_shutdown(generate_plot)
    
    # Keep the node running and wait for an interrupt
    rospy.spin()