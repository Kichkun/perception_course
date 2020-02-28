"""
Sudhanva Sreesha
ssreesha@umich.edu
24-Apr-2018

Gonzalo Ferrer
g.ferrer@skoltech.ru
26-Nov-2018
"""

import numpy as np

from tools.task import get_prediction
from slam.slamBase import SlamBase


class SimulationSlamBase(SlamBase):
    def predict(self, u, dt=None):
        iR = self.iR  # Robot indexes
        iM = self.iM  # Map indexes
        mu_r = self.mu[iR]

        F_r = self.get_g_prime_wrt_state(mu_r, u)
        F_e = self.get_g_prime_wrt_motion(mu_r, u)
        M_t = self.get_motion_noise_covariance(u)
        R_t = F_e @ M_t @ F_e.T

        # EKF prediction of the state mean.
        self.state.mu[iR, 0] = get_prediction(mu_r, u)

        # EKF prediction of the state covariance.
        iRT = iR[:, None]
        iMT = iM[:, None]

        self.state.Sigma[iRT, iR] = F_r @ self.Sigma[iRT, iR] @ F_r.T + R_t

        if iM.size > 0:
            self.state.Sigma[iRT, iM] = F_r @ self.Sigma[iRT, iM]
            self.state.Sigma[iMT, iR] = self.state.Sigma[iRT, iM].T

    def get_motion_noise_covariance(self, motion):
        """
        :param motion: The motion command at the current time step (format: [drot1, dtran, drot2]).
        :return: The covariance of the motion noise (in motion space).
        """

        drot1, dtran, drot2 = motion
        a1, a2, a3, a4 = self.params.alphas

        return np.diag([a1 * drot1 ** 2 + a2 * dtran ** 2,
                        a3 * dtran ** 2 + a4 * (drot1 ** 2 + drot2 ** 2),
                        a1 * drot2 ** 2 + a2 * dtran ** 2])

    @staticmethod
    def get_g_prime_wrt_state(state, motion):
        """
        :param state: The current state mean of the robot (format: np.array([x, y, theta])).
        :param motion: The motion command at the current time step (format: np.array([drot1, dtran, drot2])).
        :return: Jacobian of the state transition matrix w.r.t. the state.
        """

        drot1, dtran, drot2 = motion

        return np.array([[1, 0, -dtran * np.sin(state[2] + drot1)],
                         [0, 1, dtran * np.cos(state[2] + drot1)],
                         [0, 0, 1]])

    @staticmethod
    def get_g_prime_wrt_motion(state, motion):
        """
        :param state: The current state mean of the robot (format: np.array([x, y, theta])).
        :param motion: The motion command at the current time step (format: np.array([drot1, dtran, drot2])).
        :return: Jacobian of the state transition matrix w.r.t. the motion command.
        """

        drot1, dtran, drot2 = motion

        return np.array([[-dtran * np.sin(state[2] + drot1), np.cos(state[2] + drot1), 0],
                         [dtran * np.cos(state[2] + drot1), np.sin(state[2] + drot1), 0],
                         [1, 0, 1]])

