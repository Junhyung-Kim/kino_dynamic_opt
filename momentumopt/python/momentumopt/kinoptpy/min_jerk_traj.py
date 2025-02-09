'''
@file min_jerk_traj.py
@package momentumopt
@author Brahayam Ponton (brahayam.ponton@tuebingen.mpg.de)
@license License BSD-3-Clause
@copyright Copyright (c) 2019, New York University and Max Planck Gesellschaft.
@date 2019-10-08
'''

import numpy as np
import pdb


class Polynomial:

    def __init__(self, coeffs=None):
        self.fitted = False
        self.coeffs = coeffs
        if not coeffs is None:
            self.order = self.coeffs.shape[0] - 1
            self.fitted = True
        else:
            self.order = None
            self.constraints = []

    def set_initial_coeffs(self):
        if not self.order is None:
            coeff = np.zeros((self.order + 1, 2))
            coeff[:, 0] = np.ones((self.order + 1))
            coeff[:, 1] = np.cumsum(np.concatenate((np.array([0]), np.ones(self.order))))
            self.coeffs = coeff
        else:
            raise ValueError("Order of polynomial is not specified.")

    def get_coeffs(self):
        return self.coeffs

    def set_coeffs(self, coeffs):
        self.coeffs = coeffs
        self.fitted = True

    def differentiate(self):
        # pdb.set_trace()
        diff_coeff = np.zeros_like(self.coeffs)
        diff_coeff[:, 0] = self.coeffs[:, 0].copy() * self.coeffs[:, 1].copy()
        diff_coeff[:, 1] = self.coeffs[:, 1].copy() - 1
        diff_coeff[diff_coeff[:, 1] < 0, :] = 0
        return diff_coeff

    def eval(self, x):
        if not self.fitted:
            raise ValueError("Polynomial has not been fitted yet.")
        return np.sum(np.power(x, self.coeffs[:, 1]) * self.coeffs[:, 0])

    def deval(self, x):
        if not self.fitted:
            raise ValueError("Polynomial has not been fitted yet.")

        # HACK: For a constant polynominal, don't use the normal equation
        # as it leads to division by zero.
        if self.coeffs.shape[0] == 1:
            return 0.
        else:
            coeffs = self.differentiate()
            return np.sum(np.power(x, coeffs[:, 1]) * coeffs[:, 0])

    def set_constraints(self, x_values, y_values, derivative_orders):
        for i in range(len(x_values)):
            self.set_constraint(x_values[i], y_values[i], derivative_orders[i])

        self.order = len(self.constraints) - 1
        self.set_initial_coeffs()

    def set_constraint(self, x, y, derivative_order):
        self.constraints.append((x, y, derivative_order))

    def fit(self):
        if len(self.constraints) == 0:
            raise ValueError("No constraints specificed yet.")
        
        A = np.zeros((self.order + 1, self.order + 1))
        b = np.zeros((self.order + 1))

        coeffs_prime = self.differentiate()
        poly_prime = Polynomial(coeffs=coeffs_prime)
        coeffs_prime_prime = poly_prime.differentiate()

        for i, constraint in enumerate(self.constraints):
            if constraint[-1] == 0:
                A[i, :] = np.power(constraint[0], self.coeffs[:, 1]) * self.coeffs[:, 0]
            elif constraint[-1] == 1:
                A[i, :] = np.power(constraint[0], coeffs_prime[:, 1]) * coeffs_prime[:, 0]
            elif constraint[-1] == 2:
                A[i, :] = np.power(constraint[0], coeffs_prime_prime[:, 1]) * coeffs_prime_prime[:, 0]
            else:
                raise ValueError("Derivative not specificed")

            b[i] = constraint[1]
        
        x = np.linalg.solve(A, b)
        
        self.coeffs[:, 0] = x
        self.fitted = True


def create_constraints(t, x, via=None):
    # Generate minimum jerk trajectories for endeffector motion by
    # fitting a polynomial for every dimension (x, y, z).
    # t_0: Time when endeffector switches from being in contact to not being in contact
    # t_1: Time when endeffector switches from being not in contact to being in contact
    # Example for the endeffector motion in z:
    # f_z(t_0) = z_0
    # f_z'(t_0) = 0  # zero velocity when in contact
    # f_z"(t_0) = 0  # zero acceleration when in contact
    # f_z(0.5 * (t_1 - t_0) + t_0) = 0.5 * (z_1 - z_0) + z_0 OR f_z(0.5 * (t_1 - t_0) + t_0) = z_max
    # f_z(t_1) = z_1
    # f_z'(t_1) = 0  # zero velocity when in contact
    # f_z"(t_1) = 0  # zero acceleration when in contact

    t_0 = t[0]
    t_1 = t[1]
    x_0 = x[0]
    x_1 = x[1]

    t_center = 0.5 * (t_1 - t_0) + t_0

    if via is None:
        via = 0.5 * (x_1 - x_0) + x_0

    constraints = np.zeros((7, 3))

    # constraints[i, :] = [t, x, order of derivative]
    constraints[0, :] = [t_0, x_0, 0]
    constraints[1, :] = [t_0, 0.0, 1]
    constraints[2, :] = [t_0, 0.0, 2]
    constraints[3, :] = [t_center, via, 0]
    constraints[4, :] = [t_1, x_1, 0]
    constraints[5, :] = [t_1, 0.0, 1]
    constraints[6, :] = [t_1, 0.0, 2]

    return constraints

def create_constraints1(t, x, y, z, t1, via=None):
    # Generate minimum jerk trajectories for endeffector motion by
    # fitting a polynomial for every dimension (x, y, z).
    # t_0: Time when endeffector switches from being in contact to not being in contact
    # t_1: Time when endeffector switches from being not in contact to being in contact
    # Example for the endeffector motion in z:
    # f_z(t_0) = z_0
    # f_z'(t_0) = 0  # zero velocity when in contact
    # f_z"(t_0) = 0  # zero acceleration when in contact
    # f_z(0.5 * (t_1 - t_0) + t_0) = 0.5 * (z_1 - z_0) + z_0 OR f_z(0.5 * (t_1 - t_0) + t_0) = z_max
    # f_z(t_1) = z_1
    # f_z'(t_1) = 0  # zero velocity when in contact
    # f_z"(t_1) = 0  # zero acceleration when in contact

    t_0 = t[0]
    t_1 = t[1]
    x_0 = x[0]
    x_1 = x[1]
    y_0 = y[0]
    y_1 = y[1]
    z_0 = z[0]
    z_1 = z[1]

    constraints = np.zeros((6, 7))

    # constraints[i, :] = [t, x, order of derivative]
    constraints[0, :] = [t_0, x_0, y_0, z_0, 0, t1[0], t1[1]]
    constraints[1, :] = [t_0, 0.0, 0.0, 0.0, 1, t1[0], t1[1]]
    constraints[2, :] = [t_0, 0.0, 0.0, 0.0, 2, t1[0], t1[1]]
    constraints[3, :] = [t_1, x_1,  y_1, z_1, 0, t1[0], t1[1]]
    constraints[4, :] = [t_1, 0.0, 0.0, 0.0, 1, t1[0], t1[1]]
    constraints[5, :] = [t_1, 0.0, 0.0, 0.0, 2, t1[0], t1[1]]

    return constraints


def constant_poly(y_const):
    coeffs_ = np.zeros((1, 2))
    coeffs_[0, 0] = y_const

    return Polynomial(coeffs=coeffs_)


def poly_points(t, y_from, y_to, via=None):
    constraints = create_constraints(t, [y_from, y_to], via=via)
    poly = Polynomial()
    poly.set_constraints(constraints[:, 0], constraints[:, 1], constraints[:, 2])
    poly.fit()
    return poly

def poly_points1(t, x_from, x_to, y_from, y_to, z_from, z_to, t1, via=None):
    constraints = create_constraints1(t, [x_from, x_to], [y_from, y_to], [z_from, z_to], t1, via=via)
    return constraints

class PolynominalList(object):
    '''
    This class holds a list of polynominals over different time windows
    and for a given time evaluates the corresponding polynominal.
    '''
    def __init__(self):
        self.polynominals = []
        self.times = []

    def append(self, t, poly):
        self.times.append(t)
        self.polynominals.append(poly)

    def get_poly(self, t):
        i = 0
        while i < len(self.times) - 1 and self.times[i][1] < t:
            i += 1

        return self.polynominals[i]

    def eval(self, t):
        return self.get_poly(t).eval(t)

    def deval(self, t):
        return self.get_poly(t).deval(t)