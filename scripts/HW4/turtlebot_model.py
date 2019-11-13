import numpy as np

EPSILON_OMEGA = 1e-3

def compute_dynamics(x, u, dt, compute_jacobians=True):
    """
    Compute Turtlebot dynamics (unicycle model).

    Inputs:
                        x: np.array[3,] - Turtlebot state (x, y, theta).
                        u: np.array[2,] - Turtlebot controls (V, omega).
        compute_jacobians: bool         - compute Jacobians Gx, Gu if true.
    Outputs:
         g: np.array[3,]  - New state after applying u for dt seconds.
        Gx: np.array[3,3] - Jacobian of g with respect to x.
        Gu: np.array[3,2] - Jacobian of g with respect ot u.
    """
    ########## Code starts here ##########
    # TODO: Compute g, Gx, Gu
    V, omega = u
    _x, _y, _theta = x

    if abs(omega) > EPSILON_OMEGA:
        g = x + np.array([
            (V/omega) * (np.sin(_theta+omega*dt) - np.sin(_theta)),
            (V/omega) * (-np.cos(_theta+omega*dt) + np.cos(_theta)),
            omega*dt,
        ])

        Gx = np.eye(3)
        Gx[0,2] = V/omega * (np.cos(_theta+omega*dt) - np.cos(_theta))
        Gx[1,2] = V/omega * (np.sin(_theta+omega*dt) - np.sin(_theta))

        Gu = np.zeros((3,2))
        Gu[0,0] = 1/omega * (np.sin(_theta+omega*dt) - np.sin(_theta))
        Gu[1,0] = 1/omega * (-np.cos(_theta+omega*dt) + np.cos(_theta))
        Gu[2,0] = 0

        Gu[0,1] = V*(dt * np.cos(_theta+omega*dt) * omega -  (np.sin(_theta+omega*dt) - np.sin(_theta)) )/omega**2
        Gu[1,1] = V*(dt * np.sin(_theta+omega*dt) * omega -  (-np.cos(_theta+omega*dt) + np.cos(_theta)) )/omega**2
        Gu[2,1] = dt

    else:
        # using de L'Hopital rule
        g = x + np.array([
            V*(dt * np.cos(_theta+omega*dt) ),
            V*(dt * np.sin(_theta+omega*dt) ),
            omega*dt,
        ])

        Gx = np.eye(3)
        Gx[0,2] = V * dt * (-np.sin(_theta+omega*dt))
        Gx[1,2] = V * dt * (np.cos(_theta+omega*dt))

        Gu = np.zeros((3,2))
        Gu[0,0] = dt * np.cos(_theta+omega*dt)
        Gu[1,0] = dt * np.sin(_theta+omega*dt)
        Gu[2,0] = 0

        Gu[0,1] = -0.5*V*(dt**2)*np.sin(_theta+omega*dt)
        Gu[1,1] = 0.5*V*(dt**2)*np.cos(_theta+omega*dt)
        Gu[2,1] = dt



    ########## Code ends here ##########

    if not compute_jacobians:
        return g

    return g, Gx, Gu

def transform_line_to_scanner_frame(line, x, tf_base_to_camera, compute_jacobian=True):
    """
    Given a single map line in the world frame, outputs the line parameters
    in the scanner frame so it can be associated with the lines extracted
    from the scanner measurements.

    Input:
                     line: np.array[2,] - map line (alpha, r) in world frame.
                        x: np.array[3,] - pose of base (x, y, theta) in world frame.
        tf_base_to_camera: np.array[3,] - pose of camera (x, y, theta) in base frame.
         compute_jacobian: bool         - compute Jacobian Hx if true.
    Outputs:
         h: np.array[2,]  - line parameters in the scanner (camera) frame.
        Hx: np.array[2,3] - Jacobian of h with respect to x.
    """
    alpha, r = line

    ########## Code starts here ##########
    # TODO: Compute h, Hx
    x_cam, y_cam, th_cam = tf_base_to_camera
    x_hat, y_hat, th_hat = x

    h = np.array([
        alpha - (th_hat + th_cam), 
        r - x_hat*np.cos(alpha) - y_hat*np.sin(alpha) - x_cam*np.cos(alpha-th_hat) - y_cam*np.sin(alpha-th_hat),
    ])

    Hx = np.zeros((2,3))
    Hx[0,2] = -1
    Hx[1,0] = -np.cos(alpha)
    Hx[1,1] = -np.sin(alpha)

    Hx[1,2] = -x_cam * np.sin(alpha-th_hat) + y_cam*np.cos(alpha-th_hat)

    ########## Code ends here ##########

    if not compute_jacobian:
        return h

    return h, Hx


def normalize_line_parameters(h, Hx=None):
    """
    Ensures that r is positive and alpha is in the range [-pi, pi].

    Inputs:
         h: np.array[2,]  - line parameters (alpha, r).
        Hx: np.array[2,n] - Jacobian of line parameters with respect to x.
    Outputs:
         h: np.array[2,]  - normalized parameters.
        Hx: np.array[2,n] - Jacobian of normalized line parameters. Edited in place.
    """
    alpha, r = h
    if r < 0:
        alpha += np.pi
        r *= -1
        if Hx is not None:
            Hx[1,:] *= -1
    alpha = (alpha + np.pi) % (2*np.pi) - np.pi
    h = np.array([alpha, r])

    if Hx is not None:
        return h, Hx
    return h
