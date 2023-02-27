from typing import List, Optional
import numpy as np
import csv
from scipy.linalg import svd
import pandas as pd
import matplotlib.pyplot as plt
# from mpl_toolkits import mplot3d
# Use the qt backend for Matplotlib
#%matplotlib qt

def get_laser_data(csv_path, to_numpy=True):
    # Read the CSV file into a pandas DataFrame using the ', ' delimiter
    df = pd.read_csv(csv_path, delimiter=', ')
    # Convert the values in the DataFrame to numerical data types
    df = df.replace(',', '.', regex=True)
    df = df.apply(pd.to_numeric, errors='coerce')

    if to_numpy:
        # Convert the DataFrame to a NumPy array
        return df.to_numpy()

    return df


def get_vector_dir(data, n=1):
    if n is None:
        n = np.shape(data)[1]
    # Perform singular value decomposition
    U, S, V = svd(data)
    sorted_idxs=np.argpartition(S, -n)

    # The direction of the main vector is given by the column of V with the highest singular value
    # vector_direction = V[:, np.argmax(S)]
    vectors = []
    for i in range(n):
        print(f"S={S[sorted_idxs[-i-1]]}")
        vector_direction = V[sorted_idxs[-i-1],:]
        vectors.append(vector_direction)

    print(f"{vectors=}")
    print(V[:, np.argmax(S)])
    print(vector_direction)

    return vectors

def get_axes_from_data(data_axs: List[np.ndarray]):
    """Get the axis from the measurement points. Longest measured distance is 1. axis"""
    data_all=np.zeros((0,data_axs[0].shape[-1]))
    for data in data_axs:
        data=data-data[0]
        data_all=np.concatenate((data_all, data), axis=0)
    return get_vector_dir(data_all[:, :3], n=None)

def get_transformation(x_axis, z_axis):
    y = np.cross(z_axis, x_axis)
    T = np.column_stack((x_axis, y, z_axis))
    return T

def get_rotation(ex,ey,ez):
    rotation_matrix = np.stack((ex, ey, ez), axis=1)
    return rotation_matrix

def get_y(ex,ez):
    y = np.cross(ex, ez)
    return y

def plot3d(data):
    # Erstelle das 3D-Plot
    fig = plt.figure()
    ax = plt.axes(projection='3d')
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.set_aspect('equal', adjustable='box')
    ax.scatter3D(data[:,0],data[:,1],data[:,2])
    return fig, ax

def prepData(data, rot, min_data:Optional[np.ndarray]=None):
    if min_data is None:
        min_data = data[0,:]
    data-=min_data
    data[:,:3] = data[:, :3] @ rot
    # points_min -= points_min[0, :]
    return data

def split_data(datas, t_singularities, twice=False):
    """splits data by begin/end singularities

    Args:
        datas (list(np.ndarray[:,4])): x,y,z,t
        t_singularities (list(np.ndarray[:])): times when in singularities
        twice (bool, optional): split in before, in and after OLMM. Defaults to False.

    Returns:
        (datas_vor, datas_sing, datas_nach) if twice else (datas_vor, datas_nach)
    """
    datas_vor=[]
    datas_nach=[]
    datas_sing=[]
    for i,data in enumerate(datas):
        t_begin = t_singularities[i][0] if t_singularities is not None and len(t_singularities[i]) else None
        if t_begin is None:
            t_begin = data[:,3].max()
        # split data
        data_vor = data[data[:,3]<=t_begin]
        data_nach = data[data[:,3]>t_begin]
        if twice:
            t_end=t_singularities[i][-1] if t_singularities is not None and len(t_singularities[i])>1 else None
            if t_end is None:
                t_end = data[:,3].max()
            data_sing = data_nach[[data_nach[:,3]<t_end]]
            data_nach = data_nach[[data_nach[:,3]>=t_end]]
        datas_vor.append(data_vor)
        datas_nach.append(data_nach)
        datas_sing.append(data_sing)

    if twice:
        return datas_vor, datas_sing, datas_nach
    return datas_vor, datas_nach

# Standardabweichung
def calc_var(data):
    # print(f"{len(data)=}")
    # Mittelwert
    m = np.mean(data[:,:3], axis=0)
    #Varianz
    v = np.var(data[:,:3], axis=0)
    #Standardabweichung
    s = np.std(data[:,:3], axis=0)
    return m, v, s

def write_vars(f, datas, names, datas_vor, datas_nach, datas_in=None):
    for i, data in enumerate(datas):
        f.write(f"{names[i]=}, {len(data)=}\n")
        m, v, s = calc_var(data)
        f.write(f"{m=} {v=} {s=}\n")

        if datas_vor is not None:
            # print("VOR")
            m, v, s = calc_var(datas_vor[i])
            f.write(f"vor:, {len(datas_vor[i])=}\n")
            f.write(f"{m=} {v=} {s=}\n")

        if datas_in is not None:
            # print("IN")
            m, v, s = calc_var(datas_in[i])
            f.write(f"in:, {len(datas_in[i])=}\n")
            f.write(f"{m=} {v=} {s=}\n")
        
        if datas_nach is not None:
            # print("NACH")
            m, v, s = calc_var(datas_nach[i])
            f.write(f"nach:, {len(datas_nach[i])=}\n")
            f.write(f"{m=} {v=} {s=}\n")
            f.write(f"\n")

def write_vars_latex(f, datas, names, datas_vor, datas_nach, datas_in=None):
    def write_vectors(f,x, name):
        f.write(f"{name}\n")
        axis=["x","y","z"]
        for i in range(len(x[0])):
            f.write(f"{axis[i]} \n")
            res=""
            for j in range(len(x)):
                res+="{"+f"{x[j][i]}"+"}"
            f.write(f"{res} \n")

    for i, data in enumerate(datas):
        interval_names=[]
        means, vars, stds = [], [], []
        if datas is not None:
            m, v, s = calc_var(data)
            means.append(np.array(m))
            vars.append(np.array(v))
            stds.append(np.array(s))
            interval_names.append("all")
        
        if datas_vor[i] is not None:
            m, v, s = calc_var(datas_vor[i])
            means.append(np.array(m))
            vars.append(np.array(v))
            stds.append(np.array(s))
            interval_names.append("vor")

        if datas_in is not None:
            m, v, s = calc_var(datas_in[i])
            means.append(np.array(m))
            vars.append(np.array(v))
            stds.append(np.array(s))
            interval_names.append("in")

        if datas_nach[i] is not None:
            m, v, s = calc_var(datas_nach[i])
            means.append(np.array(m))
            vars.append(np.array(v))
            stds.append(np.array(s))
            interval_names.append("nach")

        f.write(f"{names[i]} & ")
        # write {all}{vor}{in}{nach} for means, vars and stds:
        write_vectors(f,means, "Means")
        write_vectors(f,vars, "Vars")
        write_vectors(f,stds, "Stds")
        f.write(f"\n")

if __name__ == "__main__":
    # Path to CSV file with data from the Z axis
    csv_path_z = r"C:\Users\hauke\OneDrive\Desktop\ks_z.csv"
    csv_path_x = r"C:\Users\hauke\OneDrive\Desktop\ks_x.csv"

    data_z=get_laser_data(csv_path_z)
    data_x = get_laser_data(csv_path_x)
    points_x=data_x[:,:3]
    points_z = data_z[:,:3]

    axs=get_axes_from_data([points_x,points_z])
    axs[0]=-axs[0]
    print(f"{axs=}")
    # y=np.cross(axs[0], axs[1])
    # T = np.column_stack((axs[1],y, axs[0]))
    T=get_transformation(axs[1], axs[0])

    # p=points_x@T
    p=points_x@T.T

    all_points=np.concatenate(points_x, points_z, axis=0)
    _, ax=plot3d(all_points[:, :3])
    ax.quiver(0, 0, 0, axs[0][0], axs[0][1], axs[0][2], color='blue')
    plt.show()

    print("Finished")