import numpy as np
import scipy.interpolate

def compute_smoothed_traj(path, V_des, alpha, dt):
    """
    Fit cubic spline to a path and generate a resulting trajectory for our
    wheeled robot.

    Inputs:
        path (np.array [N,2]): Initial path
        V_des (float): Desired nominal velocity, used as a heuristic to assign nominal
            times to points in the initial path
        alpha (float): Smoothing parameter (see documentation for
            scipy.interpolate.splrep)
        dt (float): Timestep used in final smooth trajectory
    Outputs:
        traj_smoothed (np.array [N,7]): Smoothed trajectory
        t_smoothed (np.array [N]): Associated trajectory times
    Hint: Use splrep and splev from scipy.interpolate
    """
    ########## Code starts here ##########
    current_plan = np.array(path)

    path_t = [0]
    path_x = [current_plan[0][0]]
    path_y = [current_plan[0][1]]
    for i in range(current_plan.shape[0]-1):
        dx = current_plan[i+1][0]-current_plan[i][0]
        dy = current_plan[i+1][1]-current_plan[i][1]
        path_t.append(path_t[-1] + np.sqrt(dx**2 + dy**2) / V_des)
        path_x.append(current_plan[i+1][0])
        path_y.append(current_plan[i+1][1])


    # interpolate the path with cubic spline
    path_x_spline = scipy.interpolate.splrep(path_t, path_x, k=3, s=alpha)
    path_y_spline = scipy.interpolate.splrep(path_t, path_y, k=3, s=alpha)
    path_tf = path_t[-1]

    t_smoothed = np.arange(0, path_tf+0.001, dt)
    # t_smoothed=path_t
    x_d = scipy.interpolate.splev(t_smoothed, path_x_spline, der=0)
    y_d = scipy.interpolate.splev(t_smoothed, path_y_spline, der=0)
    xd_d = scipy.interpolate.splev(t_smoothed, path_x_spline, der=1)
    yd_d = scipy.interpolate.splev(t_smoothed, path_y_spline, der=1)
    xdd_d = scipy.interpolate.splev(t_smoothed, path_x_spline, der=2)
    ydd_d = scipy.interpolate.splev(t_smoothed, path_y_spline, der=2)
    theta_d = np.arctan2(yd_d,xd_d)

    traj_smoothed = np.vstack((x_d, y_d, theta_d, xd_d, yd_d, xdd_d, ydd_d)).T
    
    ########## Code ends here ##########

    return traj_smoothed, t_smoothed