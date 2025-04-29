import pandas as pd
import numpy as np
import matplotlib.pyplot as plt


df_odom    = pd.read_csv('3dramprotationodom.csv',     usecols=['time','x','y'])
df_amcl    = pd.read_csv('tixingrotationamcl.csv',     usecols=['time','x','y'])
df_gt_amcl = pd.read_csv('tixingrotationamclground.csv', usecols=['time','x','y'])
df_gt3d   = pd.read_csv('3dramprotationground.csv',   usecols=['time','x','y'])

for df in (df_amcl, df_gt_amcl, df_odom, df_gt3d):
    df['time_s'] = df['time'] / 1e9               
    df['t_rel']  = df['time_s'] - df['time_s'].iloc[0]


tolerance = 0.1  
df_odom_gt3d = pd.merge_asof(
    df_odom.sort_values('t_rel'),
    df_gt3d.sort_values('t_rel'),
    on='t_rel',
    suffixes=('_odom', '_gt3d'),
    tolerance=tolerance,
    direction='nearest'
).dropna()

df_amcl_gta = pd.merge_asof(
    df_amcl.sort_values('t_rel'),
    df_gt_amcl.sort_values('t_rel'),
    on='t_rel',
    suffixes=('_amcl', '_gtamcl'),
    tolerance=tolerance,
    direction='nearest'
).dropna()


df_odom_gt3d['error'] = np.hypot(
    df_odom_gt3d['x_odom'] - df_odom_gt3d['x_gt3d'],
    df_odom_gt3d['y_odom'] - df_odom_gt3d['y_gt3d']
)
df_amcl_gta['error'] = np.hypot(
    df_amcl_gta['x_amcl'] - df_amcl_gta['x_gtamcl'],
    df_amcl_gta['y_amcl'] - df_amcl_gta['y_gtamcl']
)


plt.figure(figsize=(10, 5))
plt.plot(df_odom_gt3d['t_rel'], df_odom_gt3d['error'], label='Localization', linewidth=1)
plt.plot(df_amcl_gta ['t_rel'], df_amcl_gta ['error'], label='AMCL', linewidth=1)

plt.xlabel('Time (s)')
plt.ylabel('Position Error (m)')
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()
