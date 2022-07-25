# ---------------------------------------------------------------------
# Project "Track 3D-Objects Over Time"
# Copyright (C) 2020, Dr. Antje Muntzinger / Dr. Andreas Haja.
#
# Purpose of this file : Kalman filter class
#
# You should have received a copy of the Udacity license together with this program.
#
# https://www.udacity.com/course/self-driving-car-engineer-nanodegree--nd013
# ----------------------------------------------------------------------
#

# imports
import numpy as np

# add project directory to python path to enable relative imports
import os
import sys
PACKAGE_PARENT = '..'
SCRIPT_DIR = os.path.dirname(os.path.realpath(os.path.join(os.getcwd(), os.path.expanduser(__file__))))
sys.path.append(os.path.normpath(os.path.join(SCRIPT_DIR, PACKAGE_PARENT)))
import misc.params as params 

class Filter:
    '''Kalman filter class'''
    def __init__(self):
        pass

    def F(self):
        dt = params.dt
        return np.matrix([[1, 0, 0, dt, 0, 0],
                          [0, 1, 0, 0, dt, 0],
                          [0, 0, 1, 0, 0, dt],
                          [0, 0, 0, 1, 0, 0 ],
                          [0, 0, 0, 0, 1, 0,],
                          [0, 0, 0, 0, 0, 1]])

    def Q(self):
        q = params.q
        dt = params.dt
        q3 = 1/3*(dt**3)*q
        q2 = 1/2*(dt**2)*q
        q1 = q*dt
        return np.matrix([[q3, 0, 0, q2, 0, 0],
                          [0, q3, 0, 0, q2, 0],
                          [0, 0, q3, 0, 0, q2],
                          [q2, 0, 0, q1, 0, 0],
                          [0, q2, 0, 0, q1, 0],
                          [0, 0, q2, 0, 0, q1]])


    def predict(self, track):
        # predict state x and estimation error covariance P to next timestep, save x and P in track
        F = self.F()
        Q = self.Q()
        track.set_x(F*track.x) # state prediction
        track.set_P(F*track.P*F.transpose() + Q) # covariance prediction
 
    def update(self, track, meas):
        # update state x and covariance P with associated measurement, save x and P in track
        H = meas.sensor.get_H(track.x) # calculate Jacobian H
        gamma = self.gamma(track, meas)
        S = self.S(track, meas, H) # covariance of residual
        K = track.P*H.transpose()*np.linalg.inv(S) # Kalman gain=
        track.set_x(track.x + K*gamma) # state update
        I = np.identity(params.dim_state)
        track.set_P((I - K*H) * track.P) # covariance update

        track.update_attributes(meas)
    
    def gamma(self, track, meas):
        # calculate and return residual gamma
        return meas.z - meas.sensor.get_hx(track.x)

    def S(self, track, meas, H):
        # calculate and return covariance of residual S
        return H * track.P * H.transpose() + meas.R
