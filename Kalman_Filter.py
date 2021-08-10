import numpy as np

class KalmanFilter:
    def __init__(self, processVar=10, measVar = 0.5, initial_r = ((0,0),(0,0),0)):
        self.x_initial, self.y_initial, self.w_initial, self.h_initial, self.rot_initial = convert(initial_r)
        
        self.processVar = processVar
        self.measVar = measVar

        self.Q = np.eye(5)*self.processVar
        self.R = np.array([[self.measVar, 0, 0, 0, 0], [0, self.measVar, 0, 0, 0], [0, 0, 0, 0, 0], [0, 0, 0, 0, 0], [0, 0, 0, 0, 0]])
        self.H = np.eye(5)
        self.A = np.array([[1, 0, 1, 0, 0], [0, 1, 0, 1, 0], [0, 0, 1, 0, 0], [0, 0, 0, 1, 0], [0, 0, 0, 0, 1]])

        self.cnt = 0

    def compute_Para(self, r):
        self.x, self.y, self.w, self.h, self.rot = convert(r)
        Z = np.array([[self.x, self.y, self.w, self.h, self.rot]], dtype="float").T

        if self.cnt == 0:
            self.X_estimate = np.array([[self.x_initial, self.y_initial, self.w_initial, self.h_initial, self.rot_initial]], dtype="float").T
            self.P_estimate = np.eye(5,dtype="float")

        else:
            X_predict = np.dot(self.A, self.X_estimate)
            P_predict = np.dot(np.dot(self.A, self.P_estimate),self.A.T) + self.Q

            K = (np.dot(P_predict,self.H.T)) @ np.linalg.inv(((np.dot(np.dot(self.H, P_predict),self.H.T) + self.R)))
            self.X_estimate = X_predict + K @ (Z - np.dot(self.H, X_predict))
            self.P_estimate = P_predict - K @ self.H @ P_predict 
        
        self.cnt += 1

        para = ((self.X_estimate[0][0], self.X_estimate[1][0]), (self.X_estimate[2][0], self.X_estimate[3][0]), self.X_estimate[4][0])
        
        return para


def convert(r):
    x = r[0][0]
    y = r[0][1]
    w = r[1][0]
    h = r[1][1]
    rot = r[2]
    return x,y,w,h,rot

    