import numpy as np

class EKF:
    def __init__(self, model, x0: np.ndarray, P0: np.ndarray, useJosephForm: bool = False):
        self.x0 = x0
        self.x = x0.copy()
        self.P0 = P0
        self.P = P0.copy()
        self.model = model
        self.nStates = len(x0)
        self.nObservations = len(model.getObservationMatrix(x0))
        self.K = np.zeros((self.nStates, self.nObservations))
        self.useJosephForm = useJosephForm

    def predict(self, u, Q, dt):
        F = self.model.getStateTransitionMatrix(self.x, u, dt)
        # Calculate predicted estimate covariance
        self.P = F @ self.P @ F.T + Q

        # Propagate system dynamics
        self.x = self.model.systemDynamics(self.x, u, dt)

    def update(self, y, R):
        # Calculate innovation
        innovation = np.array([y - self.model.observationFunction(self.x)]).T
        # Calculate innovation covariance
        H = self.model.getObservationMatrix(self.x)
        S = H @ self.P @ H.T + R

        # Calculate Kalman gain
        self.K = self.P @ H.T @ np.linalg.inv(S)

        # Update state estimation
        self.x += (self.K @ innovation).flatten()

        # Calculate updated estimate covariance
        I_minus_KH = np.eye(self.nStates) - self.K @ H
        if self.useJosephForm:
            self.P = I_minus_KH @ self.P @ I_minus_KH.T + self.K @ R @ self.K.T
        else:
            self.P = I_minus_KH @ self.P