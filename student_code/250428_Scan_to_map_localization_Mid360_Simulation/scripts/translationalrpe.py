import pandas as pd
import numpy as np
import matplotlib.pyplot as plt


amcl_file      = 'tixingrotationamcl.csv'
amcl_gt_file   = 'tixingrotationamclground.csv'
odom_file      = '3dramprotationodom.csv'
odom_gt_file   = '3dramprotationground.csv'


df_amcl    = pd.read_csv(amcl_file,    usecols=['time','x','y'])
df_amcl_gt = pd.read_csv(amcl_gt_file, usecols=['time','x','y'])
df_odom    = pd.read_csv(odom_file,    usecols=['time','x','y'])
df_odom_gt = pd.read_csv(odom_gt_file, usecols=['time','x','y'])


for df in (df_amcl, df_amcl_gt, df_odom, df_odom_gt):
    df['time'] = df['time'].astype(np.int64)
    df.sort_values('time', inplace=True)
    df.reset_index(drop=True, inplace=True)


tolerance = 100_000_000  
df_amcl_m = pd.merge_asof(df_amcl, df_amcl_gt, on='time', suffixes=('_amcl','_gt'),
                          direction='nearest', tolerance=tolerance).dropna().reset_index(drop=True)
df_odom_m = pd.merge_asof(df_odom, df_odom_gt, on='time', suffixes=('_odom','_gt'),
                          direction='nearest', tolerance=tolerance).dropna().reset_index(drop=True)


for df, suf in ((df_amcl_m,'amcl'), (df_odom_m,'odom')):

    df[f'dx_{suf}'] = df[f'x_{suf}'].diff()
    df[f'dy_{suf}'] = df[f'y_{suf}'].diff()
    df['dx_gt']     = df['x_gt'].diff()
    df['dy_gt']     = df['y_gt'].diff()
    # RPE
    df[f'rpe_{suf}'] = np.hypot(df[f'dx_{suf}'] - df['dx_gt'],
                                df[f'dy_{suf}'] - df['dy_gt'])

    df.loc[0, f'rpe_{suf}'] = np.nan


for df in (df_amcl_m, df_odom_m):
    df['time_s'] = df['time'] / 1e9
    df['t_rel']  = df['time_s'] - df['time_s'].iloc[0]





plt.figure(figsize=(10,4))
plt.plot(df_amcl_m['t_rel'], df_amcl_m['rpe_amcl'], label='AMCL translational RPE', linewidth=1)
plt.plot(df_odom_m['t_rel'], df_odom_m['rpe_odom'], label='Localization translational RPE', linewidth=1)
plt.xlabel('Time since start (s)')
plt.ylabel('Translational RPE (m)')
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()


