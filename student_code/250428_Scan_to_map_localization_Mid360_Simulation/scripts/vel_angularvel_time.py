import pandas as pd
import matplotlib.pyplot as plt


loc = pd.read_csv('3drectgroundtruth.csv')


cols = [
    '%time',
    'field.twist.twist.linear.x',
    'field.twist.twist.linear.y',
    'field.twist.twist.linear.z',
    'field.twist.twist.angular.x',
    'field.twist.twist.angular.y',
    'field.twist.twist.angular.z'
]
loc = loc[cols].copy()
loc.rename(columns={
    '%time': 'time_ns',
    'field.twist.twist.linear.x': 'vx',
    'field.twist.twist.linear.y': 'vy',
    'field.twist.twist.linear.z': 'vz',
    'field.twist.twist.angular.x': 'wx',
    'field.twist.twist.angular.y': 'wy',
    'field.twist.twist.angular.z': 'wz'
}, inplace=True)


loc['time_s'] = loc['time_ns'] / 1e9
loc['t_rel'] = loc['time_s'] - loc['time_s'].iloc[0]


fig, axes = plt.subplots(1, 2, figsize=(12, 4))


axes[0].plot(loc['t_rel'], loc['vx'], label='vx')
axes[0].plot(loc['t_rel'], loc['vy'], label='vy')
axes[0].plot(loc['t_rel'], loc['vz'], label='vz')
#axes[0].set_title('Localization Linear Velocity')
axes[0].set_xlabel('Time [s] (relative)')
axes[0].set_ylabel('Linear Velocity [m/s]')
axes[0].legend()


axes[1].plot(loc['t_rel'], loc['wx'], label='wx')
axes[1].plot(loc['t_rel'], loc['wy'], label='wy')
axes[1].plot(loc['t_rel'], loc['wz'], label='wz')
#axes[1].set_title('Localization Angular Velocity')
axes[1].set_xlabel('Time [s] (relative)')
axes[1].set_ylabel('Angular Velocity [rad/s]')
axes[1].legend()

plt.tight_layout()
plt.show()

