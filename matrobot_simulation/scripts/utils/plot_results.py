#!/usr/bin/env python3
import matplotlib.pyplot as plt
import pandas as pd
from utils.project_paths import ProjectPaths
def main():
    paths=ProjectPaths(); df=pd.read_csv(paths.navigation_log)
    plt.figure(); plt.plot(df['x'],df['y'],label='Estimated robot path'); plt.scatter([df['goal_x'].iloc[0]],[df['goal_y'].iloc[0]],marker='x',label='Goal'); plt.title('Robot Path'); plt.xlabel('x [m]'); plt.ylabel('y [m]'); plt.legend(); plt.grid(True); plt.axis('equal'); plt.savefig(paths.plot_dir/'navigation_xy_path.png',dpi=200)
    plt.figure(); plt.plot(df['time'],df['distance_to_goal']); plt.title('Distance to Goal Over Time'); plt.xlabel('time [s]'); plt.ylabel('distance [m]'); plt.grid(True); plt.savefig(paths.plot_dir/'distance_to_goal.png',dpi=200)
    plt.figure(); plt.plot(df['time'],df['front_distance']); plt.title('Front Obstacle Distance'); plt.xlabel('time [s]'); plt.ylabel('distance [m]'); plt.grid(True); plt.savefig(paths.plot_dir/'front_distance.png',dpi=200)
    plt.figure(); plt.plot(df['time'],df['yaw_error_to_goal']); plt.title('Goal Yaw Error'); plt.xlabel('time [s]'); plt.ylabel('yaw error [rad]'); plt.grid(True); plt.savefig(paths.plot_dir/'goal_yaw_error.png',dpi=200)
    print(f'Plots saved to: {paths.plot_dir}')
if __name__=='__main__': main()
