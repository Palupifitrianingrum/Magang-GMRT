import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation

L1 = 50.0
L2 = 40.0
L3 = 30.0
L_BASE = 20.0

def fk_2dof_simple(theta1, theta2, l1=L1, l2=L2):
    P0 = (0.0, 0.0)
    x1 = l1 * np.cos(theta1)
    y1 = l1 * np.sin(theta1)
    P1 = (x1, y1)
    x2 = x1 + l2 * np.cos(theta1 + theta2)
    y2 = y1 + l2 * np.sin(theta1 + theta2)
    P2 = (x2, y2)
    return P0, P1, P2

def create_trans_matrix(x, y, z):
    return np.array([
        [1, 0, 0, x],
        [0, 1, 0, y],
        [0, 0, 1, z],
        [0, 0, 0, 1]
    ])

def create_rot_z_matrix(theta):
    cos_t = np.cos(theta)
    sin_t = np.sin(theta)
    return np.array([
        [cos_t, -sin_t, 0, 0],
        [sin_t,  cos_t, 0, 0],
        [0,      0,     1, 0],
        [0,      0,     0, 1]
    ])

def create_rot_y_matrix(theta):
    cos_t = np.cos(theta)
    sin_t = np.sin(theta)
    return np.array([
        [cos_t,  0, sin_t, 0],
        [0,      1, 0,     0],
        [-sin_t, 0, cos_t, 0],
        [0,      0, 0,     1]
    ])

def fk_3dof_htm(theta1, theta2, theta3, l_base=L_BASE, l2=L2, l3=L3):
    T_0_1 = create_rot_z_matrix(theta1) @ create_trans_matrix(0, 0, l_base)
    T_1_2 = create_rot_y_matrix(theta2) @ create_trans_matrix(l2, 0, 0)
    T_2_3 = create_rot_y_matrix(theta3) @ create_trans_matrix(l3, 0, 0)
    
    T_0_2 = T_0_1 @ T_1_2
    T_0_3 = T_0_2 @ T_2_3
    
    P0 = (0, 0, 0)
    P1 = (T_0_1[0, 3], T_0_1[1, 3], T_0_1[2, 3])
    P2 = (T_0_2[0, 3], T_0_2[1, 3], T_0_2[2, 3])
    P3 = (T_0_3[0, 3], T_0_3[1, 3], T_0_3[2, 3])
    
    return P0, P1, P2, P3

fig_2d, ax_2d = None, None
line_2d, base_2d, ee_2d = None, None, None

def setup_plot_2d():
    global fig_2d, ax_2d, line_2d, base_2d, ee_2d
    
    fig_2d, ax_2d = plt.subplots(figsize=(7, 7))
    
    line_2d, = ax_2d.plot([], [], 'bo-', linewidth=3, markersize=10, label='Robot Links')
    base_2d, = ax_2d.plot([], [], 'rs', markersize=15, label='Base')
    ee_2d, = ax_2d.plot([], [], 'go', markersize=10, label='End-Effector')
    
    ax_2d.set_title("Animasi Robot Arm 2-DoF")
    ax_2d.set_xlabel('X (cm)')
    ax_2d.set_ylabel('Y (cm)')
    ax_2d.legend()
    ax_2d.grid(True)
    ax_2d.axis('equal')
    
    max_range = L1 + L2 + 10
    ax_2d.set_xlim(-max_range, max_range)
    ax_2d.set_ylim(-max_range, max_range)

def update_2d(frame):
    t1_rad = np.pi/2 * np.sin(frame / 50.0)
    t2_rad = np.pi/3 * np.cos(frame / 30.0)
    
    points = fk_2dof_simple(t1_rad, t2_rad)
    
    x_vals = [p[0] for p in points]
    y_vals = [p[1] for p in points]
    
    line_2d.set_data(x_vals, y_vals)
    base_2d.set_data([x_vals[0]], [y_vals[0]])
    ee_2d.set_data([x_vals[-1]], [y_vals[-1]])
   
    return line_2d, base_2d, ee_2d

def run_animation_2d():
    setup_plot_2d()
    anim = FuncAnimation(fig_2d, update_2d, frames=300, interval=34, blit=False)
    plt.show()

fig_3d, ax_3d = None, None
line_3d, base_3d, ee_3d = None, None, None

def setup_plot_3d():
    global fig_3d, ax_3d, line_3d, base_3d, ee_3d
    
    fig_3d = plt.figure(figsize=(8, 8))
    ax_3d = fig_3d.add_subplot(111, projection='3d')
    
    line_3d, = ax_3d.plot([], [], [], 'bo-', linewidth=3, markersize=10, label='Robot Links')
    base_3d, = ax_3d.plot([], [], [], 'rs', markersize=15, label='Base')
    ee_3d, = ax_3d.plot([], [], [], 'go', markersize=10, label='End-Effector')
    
    ax_3d.set_title("Tangan Buatane Palupi")
    ax_3d.set_xlabel('X (cm)')
    ax_3d.set_ylabel('Y (cm)')
    ax_3d.set_zlabel('Z (cm)')
    ax_3d.legend()
    
    max_range = L_BASE + L2 + L3
    ax_3d.set_xlim([-max_range, max_range])
    ax_3d.set_ylim([-max_range, max_range])
    ax_3d.set_zlim([0, max_range])

def update_3d(frame):
    t1_rad = np.radians(frame * 1.5)
    t2_rad = np.pi/4 + np.sin(frame / 40.0) * np.pi/4
    t3_rad = np.cos(frame / 20.0) * np.pi/6
    
    points_3d = fk_3dof_htm(t1_rad, t2_rad, t3_rad)
    
    x_vals = [p[0] for p in points_3d]
    y_vals = [p[1] for p in points_3d]
    z_vals = [p[2] for p in points_3d]
    
    line_3d.set_data(x_vals, y_vals)
    line_3d.set_3d_properties(z_vals)
    
    base_3d.set_data([x_vals[0]], [y_vals[0]])
    base_3d.set_3d_properties([z_vals[0]])
    
    ee_3d.set_data([x_vals[-1]], [y_vals[-1]])
    ee_3d.set_3d_properties([z_vals[-1]])
    
    return line_3d, base_3d, ee_3d

def run_animation_3d():
    setup_plot_3d()
    anim = FuncAnimation(fig_3d, update_3d, frames=300, interval=50, blit=False)
    plt.show()

if __name__ == "__main__":
    run_animation_2d()
    run_animation_3d()