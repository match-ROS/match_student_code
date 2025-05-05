import pandas as pd
import numpy as np
import matplotlib.pyplot as plt


files = [
    ('Line - AMCL',        'amcl.csv',                   'groundtruthamcl.csv'),
    ('Line - Localization','odom3d.csv',                'groundtruth3d.csv'),
    ('Rot - AMCL',         'tixingrotationamcl.csv',     'tixingrotationamclground.csv'),
    ('Rot - Localization','3dramprotationodom.csv',     '3dramprotationground.csv'),
    ('Liss - AMCL',        'tixinglissajouamcl.csv',     'tixinglissajouamclground.csv'),
    ('Liss - Localization','3dramplissajousodom.csv',   '3dramplissajousground.csv'),
]


tolerance = 100_000_000

all_errors = []
labels     = []

for label, pred_file, gt_file in files:

    df_p = pd.read_csv(pred_file, usecols=['time','x','y'])
    df_g = pd.read_csv(  gt_file, usecols=['time','x','y'])
    if pred_file == 'odom3d.csv':
        df_p['x'] = df_p['x'] - 3.0
        df_p['y'] = df_p['y'] - 3.0

    for df in (df_p, df_g):
        df['time'] = df['time'].astype(np.int64)
        df.sort_values('time', inplace=True)
        df.reset_index(drop=True, inplace=True)

    df = pd.merge_asof(
        df_p, df_g,
        on='time',
        suffixes=('_p','_g'),
        tolerance=tolerance,
        direction='nearest'
    ).dropna().reset_index(drop=True)

    df.sort_values('time', inplace=True)
    df['dx_p'] = df['x_p'].diff()
    df['dy_p'] = df['y_p'].diff()
    df['dx_g'] = df['x_g'].diff()
    df['dy_g'] = df['y_g'].diff()


    err = np.hypot(
        df['dx_p'] - df['dx_g'],
        df['dy_p'] - df['dy_g']
    ).iloc[1:].values

    all_errors.append(err)
    labels.append(label)


plt.figure(figsize=(10,6))
plt.boxplot(all_errors, labels=labels, showfliers=False)
plt.xticks(rotation=45, ha='right')
plt.ylabel('Translational RPE (m)')
plt.grid(axis='y', linestyle='--', alpha=0.7)
plt.tight_layout()
plt.show()

