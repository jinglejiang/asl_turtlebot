import numpy as np
import scipy.linalg  # You may find scipy.linalg.block_diag useful
import scipy.stats  # You may find scipy.stats.multivariate_normal.pdf useful
import turtlebot_model as tb

EPSILON_OMEGA = 1e-3

class ParticleFilter(object):
    """
    Base class for Monte Carlo localization and FastSLAM.

    Usage:
        pf = ParticleFilter(x0, R)
        while True:
            pf.transition_update(u, dt)
            pf.measurement_update(z, Q)
            localized_state = pf.x
    """

    def __init__(self, x0, R):
        """
        ParticleFilter constructor.

        Inputs:
            x0: np.array[M,3] - initial particle states.
             R: np.array[2,2] - control noise covariance (corresponding to dt = 1 second).
        """
        self.M = x0.shape[0]  # Number of particles
        self.xs = x0  # Particle set [M x 3]
        self.ws = np.repeat(1. / self.M, self.M)  # Particle weights (initialize to uniform) [M]
        self.R = R  # Control noise covariance (corresponding to dt = 1 second) [2 x 2]
        assert R[0,1]==0
        assert R[1,0]==0

    @property
    def x(self):
        """
        Returns the particle with the maximum weight for visualization.

        Output:
            x: np.array[3,] - particle with the maximum weight.
        """
        idx = self.ws == self.ws.max()
        x = np.zeros(self.xs.shape[1:])
        x[:2] = self.xs[idx,:2].mean(axis=0)
        th = self.xs[idx,2]
        x[2] = np.arctan2(np.sin(th).mean(), np.cos(th).mean())
        return x

    def transition_update(self, u, dt):
        """
        Performs the transition update step by updating self.xs.

        Inputs:
             u: np.array[2,] - zero-order hold control input.
            dt: float        - duration of discrete time step.
        Output:
            None - internal belief state (self.xs) should be updated.
        """
        ########## Code starts here ##########
        # TODO: Update self.xs.
        # Hint: Call self.transition_model().

        random_control_noise = np.zeros((self.M, 2))
        random_control_noise[:,0] = np.random.normal(scale=self.R[0,0], size=self.M)
        random_control_noise[:,1] = np.random.normal(scale=self.R[1,1], size=self.M)

        control = np.tile(u, (self.M, 1)) + random_control_noise
        # control[:,1] = control[:,1] % (2*np.pi) - np.pi
        # import pdb; pdb.set_trace()
        self.xs = self.transition_model(control, dt)

        ########## Code ends here ##########

    def transition_model(self, us, dt):
        """
        Propagates exact (nonlinear) state dynamics.

        Inputs:
            us: np.array[M,2] - zero-order hold control input for each particle.
            dt: float         - duration of discrete time step.
        Output:
            g: np.array[M,3] - result of belief mean for each particle
                               propagated according to the system dynamics with
                               control u for dt seconds.
        """
        raise NotImplementedError("transition_model must be overridden by a subclass of EKF")

    def measurement_update(self, z_raw, Q_raw):
        """
        Updates belief state according to the given measurement.

        Inputs:
            z_raw: np.array[2,I]   - matrix of I columns containing (alpha, r)
                                     for each line extracted from the scanner
                                     data in the scanner frame.
            Q_raw: [np.array[2,2]] - list of I covariance matrices corresponding
                                     to each (alpha, r) column of z_raw.
        Output:
            None - internal belief state (self.x, self.ws) is updated in self.resample().
        """
        raise NotImplementedError("measurement_update must be overridden by a subclass of EKF")

    def resample(self, xs, ws):
        """
        Resamples the particles according to the updated particle weights.

        Inputs:
            xs: np.array[M,3] - matrix of particle states.
            ws: np.array[M,]  - particle weights.

        Output:
            None - internal belief state (self.xs, self.ws) should be updated.
        """
        r = np.random.rand() / self.M

        ########## Code starts here ##########
        # TODO: Update self.xs, self.ws.
        # Note: Assign the weights in self.ws to the corresponding weights in ws
        #       when resampling xs instead of resetting them to a uniform
        #       distribution. This allows us to keep track of the most likely
        #       particle and use it to visualize the robot's pose with self.x.
        # Hint: To maximize speed, try to implement the resampling algorithm
        #       without for loops. You may find np.linspace(), np.cumsum(), and
        #       np.searchsorted() useful. This results in a ~10x speedup.

        # i, c = 0, ws[0]
        # X = np.zeros_like(xs)
        # Ws = np.zeros_like(ws)

        # for m in range(self.M):
        #     u = ws.sum() * (r + 1.0*m / self.M)
        #     while c<u:
        #         i += 1
        #         c += ws[i]

        #     X[m,:] = xs[i,:]
        #     Ws[m] = ws[i]

        # self.xs = X
        # self.ws = Ws

        ######## VECTORIZED ############
        edges = np.linspace(0, ws.sum(), num=self.M, endpoint=False)
        start = ws.sum() * r

        to_be_inserted = np.cumsum(ws)
        idx = np.searchsorted(to_be_inserted, edges+start)


        X = xs[idx,:]
        Ws = ws[idx]

        self.xs = X
        self.ws = Ws




        ########## Code ends here ##########

    def measurement_model(self, z_raw, Q_raw):
        """
        Converts raw measurements into the relevant Gaussian form (e.g., a
        dimensionality reduction).

        Inputs:
            z_raw: np.array[2,I]   - I lines extracted from scanner data in
                                     columns representing (alpha, r) in the scanner frame.
            Q_raw: [np.array[2,2]] - list of I covariance matrices corresponding
                                     to each (alpha, r) column of z_raw.
        Outputs:
            z: np.array[2I,]   - joint measurement mean.
            Q: np.array[2I,2I] - joint measurement covariance.
        """
        raise NotImplementedError("measurement_model must be overridden by a subclass of EKF")


class MonteCarloLocalization(ParticleFilter):

    def __init__(self, x0, R, map_lines, tf_base_to_camera, g):
        """
        MonteCarloLocalization constructor.

        Inputs:
                       x0: np.array[M,3] - initial particle states.
                        R: np.array[2,2] - control noise covariance (corresponding to dt = 1 second).
                map_lines: np.array[2,J] - J map lines in columns representing (alpha, r).
        tf_base_to_camera: np.array[3,]  - (x, y, theta) transform from the
                                           robot base to camera frame.
                        g: float         - validation gate.
        """
        self.map_lines = map_lines  # Matrix of J map lines with (alpha, r) as columns
        self.tf_base_to_camera = tf_base_to_camera  # (x, y, theta) transform
        self.g = g  # Validation gate
        super(self.__class__, self).__init__(x0, R)

    def transition_model(self, us, dt):
        """
        Unicycle model dynamics.

        Inputs:
            us: np.array[M,2] - zero-order hold control input for each particle.
            dt: float         - duration of discrete time step.
        Output:
            g: np.array[M,3] - result of belief mean for each particle
                               propagated according to the system dynamics with
                               control u for dt seconds.
        """

        ########## Code starts here ##########
        # TODO: Compute g.
        # Hint: We don't need Jacobians for particle filtering.
        # Hint: To maximize speed, try to compute the dynamics without looping
        #       over the particles. If you do this, you should implement
        #       vectorized versions of the dynamics computations directly here
        #       (instead of modifying turtlebot_model). This results in a
        #       ~10x speedup.

        # g = np.zeros((self.M, 3))
        # for _ in range(self.M):
        #     g[_, :] = tb.compute_dynamics(self.xs[_,:], us[_,:], dt, compute_jacobians=False)


        ############## VECTORIZED ###############
        g = np.copy(self.xs)

        # first update everything, then we overwrite small omega entries
        _v, _omega = us[:,0], us[:,1]
        _theta = self.xs[:, 2]

        large_omega_row = _omega>1e-3
        small_omega_row = _omega<=1e-3
        
        g[:,2] += _omega * dt

        g[large_omega_row, 0] += _v[large_omega_row] / _omega[large_omega_row] * (
            np.sin(_theta[large_omega_row]+_omega[large_omega_row]*dt) - np.sin(_theta[large_omega_row])
        )
        g[large_omega_row, 1] += _v[large_omega_row] / _omega[large_omega_row] * (
            -np.cos(_theta[large_omega_row]+_omega[large_omega_row]*dt) + np.cos(_theta[large_omega_row])
        )
        
        
        g[small_omega_row, 0] += _v[small_omega_row] * dt * np.cos(_theta[small_omega_row]+_omega[small_omega_row]*dt)
        g[small_omega_row, 1] += _v[small_omega_row] * dt * np.sin(_theta[small_omega_row]+_omega[small_omega_row]*dt)


        ########## Code ends here ##########

        return g

    def measurement_update(self, z_raw, Q_raw):
        """
        Updates belief state according to the given measurement.

        Inputs:
            z_raw: np.array[2,I]   - matrix of I columns containing (alpha, r)
                                     for each line extracted from the scanner
                                     data in the scanner frame.
            Q_raw: [np.array[2,2]] - list of I covariance matrices corresponding
                                     to each (alpha, r) column of z_raw.
        Output:
            None - internal belief state (self.x, self.ws) is updated in self.resample().
        """
        xs = np.copy(self.xs)
        ws = np.zeros_like(self.ws)

        ########## Code starts here ##########
        # TODO: Compute new particles (xs, ws) with updated measurement weights.
        # Hint: To maximize speed, implement this without looping over the
        #       particles. You may find scipy.stats.multivariate_normal.pdf()
        #       useful.

        # ws should be of size [M*1]
        I = z_raw.shape[1]
        _vs, _Q = self.measurement_model(z_raw, Q_raw)
        ws = scipy.stats.multivariate_normal.pdf(_vs, mean=np.zeros(2*I), cov=_Q)

        

        ########## Code ends here ##########

        self.resample(xs, ws)

    def measurement_model(self, z_raw, Q_raw):
        """
        Assemble one joint measurement and covariance from the individual values
        corresponding to each matched line feature for each particle.

        Inputs:
            z_raw: np.array[2,I]   - I lines extracted from scanner data in
                                     columns representing (alpha, r) in the scanner frame.
            Q_raw: [np.array[2,2]] - list of I covariance matrices corresponding
                                     to each (alpha, r) column of z_raw.
        Outputs:
            z: np.array[M,2I]  - joint measurement mean for M particles.
            Q: np.array[2I,2I] - joint measurement covariance.
        """
        vs = self.compute_innovations(z_raw, np.array(Q_raw))

        ########## Code starts here ##########
        # TODO: Compute Q.
        I = z_raw.shape[1]
        Q = np.zeros((2*I, 2*I))
        for i in range(I):
            Q[2*i: 2*i+2, 2*i: 2*i+2] = Q_raw[i]

        ########## Code ends here ##########

        return vs, Q

    def compute_innovations(self, z_raw, Q_raw):
        """
        Given lines extracted from the scanner data, tries to associate each one
        to the closest map entry measured by Mahalanobis distance.

        Inputs:
            z_raw: np.array[2,I]   - I lines extracted from scanner data in
                                     columns representing (alpha, r) in the scanner frame.
            Q_raw: np.array[I,2,2] - I covariance matrices corresponding
                                     to each (alpha, r) column of z_raw.
        Outputs:
            vs: np.array[M,2I] - M innovation vectors of size 2I
                                 (predicted map measurement - scanner measurement).
        """
        def angle_diff(a, b):
            a = a % (2. * np.pi)
            b = b % (2. * np.pi)
            diff = a - b
            if np.size(diff) == 1:
                if np.abs(a - b) > np.pi:
                    sign = 2. * (diff < 0.) - 1.
                    diff += sign * 2. * np.pi
            else:
                idx = np.abs(diff) > np.pi
                sign = 2. * (diff[idx] < 0.) - 1.
                diff[idx] += sign * 2. * np.pi
            return diff

        ########## Code starts here ##########
        # TODO: Compute vs (with shape [M x I x 2]).
        # Hint: To maximize speed, try to eliminate all for loops, or at least
        #       for loops over J. It is possible to solve multiple systems with
        #       np.linalg.solve() and swap arbitrary axes with np.transpose().
        #       Eliminating loops over J results in a ~10x speedup.
        #       Eliminating loops over I results in a ~2x speedup.
        #       Eliminating loops over M results in a ~5x speedup.
        #       Overall, that's 100x!

        hs = self.compute_predicted_measurements()

        # J = hs.shape[2]
        # # i will try eliminate loops over J
        # I = z_raw.shape[1]

        # vs = np.zeros((self.M, I, 2))

        # for m in range(self.M):
        #     for i in range(I):
        #         # v_ij[-1] = angle_diff(z_raw[-1,i], hs[-1,j])
        #         v_ij = z_raw[:,i].reshape((2,1)) - hs[m,:,:]
        #         v_ij[1,:] = angle_diff(z_raw[1,i]*np.ones((J)), hs[m,1,:])

        #         d_ij = np.diag(
        #             v_ij.T.dot(np.linalg.inv(Q_raw[i,:,:])).dot(v_ij)
        #         )

        #         j_best = np.argmin(d_ij)
        #         vs[m,i,:] = v_ij[:, j_best]

        ################# here is an attempt on vetorizing both J and M
        J = hs.shape[2]
        I = z_raw.shape[1]
        M = self.M
        vs = np.zeros((M, I, 2))
        for i in range(I):
            v_i_jm = np.tile(z_raw[:,i].reshape((2,1)), (M, 1, 1)) - hs # this is [M*2*1] - [M*2*J] == [M*2*J]
            v_i_jm[:, 1,:] = angle_diff(z_raw[1,i]*np.ones((M, J)), hs[:,1,:]) # v_i_jm is of [M*2*J]
            v_ij = np.transpose(v_i_jm, (1,0,2)).reshape(2,-1)

            d_ij = np.diag(
                    v_ij.T.dot(np.linalg.inv(Q_raw[i,:,:])).dot(v_ij)
                )
            d_ij = d_ij.reshape(M,J)  # d_ij is 
            j_best = np.argmin(d_ij, axis=1) # j_best is of shape [M,]

            j_best_unravel = j_best + np.arange(0,M*J, J)


            # vs is shape[M*I*2]
            vs[:,i,:] = v_ij[:, j_best_unravel].T



        ########## Code ends here ##########

        # Reshape [M x I x 2] array to [M x 2I]
        return vs.reshape((self.M,-1))  # [M x 2I]

    def compute_predicted_measurements(self):
        """
        Given a single map line in the world frame, outputs the line parameters
        in the scanner frame so it can be associated with the lines extracted
        from the scanner measurements.

        Input:
            None
        Output:
            hs: np.array[M,2,J] - J line parameters in the scanner (camera) frame for M particles.
        """
        ########## Code starts here ##########
        # TODO: Compute hs.
        # Hint: We don't need Jacobians for particle filtering.
        # Hint: To maximize speed, try to compute the predicted measurements
        #       without looping over the map lines. You can implement vectorized
        #       versions of turtlebod_model functions directly here. This
        #       results in a ~10x speedup.

        #vectorized over J
        # hs = np.zeros((self.M, 2, self.map_lines.shape[1]))

        # x_cam, y_cam, th_cam = self.tf_base_to_camera
        # J = self.map_lines.shape[1]

        # for m in range(self.M):
        #     x_hat, y_hat, th_hat = self.xs[m,:]


        #     alphas = self.map_lines[0,:]
        #     rs = self.map_lines[1,:]

        #     cos_alpha = x_hat * ( np.vstack((np.zeros(J), np.cos(alphas))) )
        #     sin_alpha = y_hat * ( np.vstack((np.zeros(J), np.sin(alphas))) )

        #     cos_alpha_minus_theta = x_cam * ( np.vstack((np.zeros(J), np.cos(alphas-th_hat))) )
        #     sin_alpha_minus_theta = y_cam * ( np.vstack((np.zeros(J), np.sin(alphas-th_hat))) )

        #     h = self.map_lines -\
        #         np.vstack((np.ones(J)*(th_hat+th_cam), np.zeros(J))) -\
        #         cos_alpha - sin_alpha - cos_alpha_minus_theta - sin_alpha_minus_theta

        #     # now lets normalize h to Ensures that r is positive and alpha is in the range [-pi, pi].
        #     mask = np.zeros(J)
        #     mask[h[1,:]<0] = 1
        #     h[0,:] += np.pi * mask
        #     h[1,:] = np.abs(h[1,:])

        #     h[0,:] = (h[0,:] + np.pi) % (2*np.pi) - np.pi

        #     hs[m, :, :] = h


        # now challenge on vectorizing over both M and J
        J = self.map_lines.shape[1]
        M = self.M
        hs = np.zeros((M, 2, J))

        x_cam, y_cam, th_cam = self.tf_base_to_camera


        # self,map_lines is of shape [2*J]
        hs = np.tile(self.map_lines, (M,1,1))

        x_hat, y_hat, th_hat = self.xs[:,0], self.xs[:,1], self.xs[:,2] # all of shape [M,]

        alphas = self.map_lines[0,:] 
        rs = self.map_lines[1,:]    # both of shape [J,]

        # shape [M*J] update
        hs[:,0,:] -= np.tile(
            (th_hat + th_cam), (J,1)
        ).T

        cos_alpha = np.tile(x_hat, (J,1)).T * np.cos(np.tile(alphas,(M,1))) # of shape [M*J]
        sin_alpha = np.tile(y_hat, (J,1)).T * np.sin(np.tile(alphas,(M,1))) # of shape [M*J]


        # x_cam*np.cos(alpha-th_hat)
        alpha_minus_theta = np.tile(alphas,(M,1)) - np.tile(th_hat, (J,1)).T
        cos_alpha_minus_theta = x_cam * np.cos(alpha_minus_theta) # of shape [M*J]
        sin_alpha_minus_theta = y_cam * np.sin(alpha_minus_theta) # of shape [M*J]

        hs[:,1,:] -= (cos_alpha+sin_alpha+cos_alpha_minus_theta+sin_alpha_minus_theta) # again, hs is of shape (M,2,J)

        # now lets normalize h to Ensures that r is positive and alpha is in the range [-pi, pi].
        mask = np.zeros((M,J))
        mask[hs[:,1,:]<0] = 1
        hs[:,0,:] += np.pi * mask
        hs[:,1,:] = np.abs(hs[:,1,:])

        hs[:,0,:] = (hs[:,0,:] + np.pi) % (2*np.pi) - np.pi


        ########## Code ends here ##########

        return hs

