import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv('tixingrotationamcl_errors.csv', usecols=['theta_abs_rad','theta_rpe_rad'])

abs_vals = df['theta_abs_rad'].dropna()
rpe_vals = df['theta_rpe_rad'].dropna()

plt.figure(figsize=(6, 4))
plt.boxplot(
    [abs_vals, rpe_vals],
    labels=['Absolute Error', 'RPE'],
    showfliers=False  
)
plt.ylabel('Error (rad)')
#plt.title('Boxplot of Orientation Errors')
plt.grid(True, axis='y', linestyle='--', alpha=0.6)
plt.tight_layout()
plt.show()

