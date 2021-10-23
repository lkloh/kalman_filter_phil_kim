#!/usr/bin/env python3
import math

GRAVITATIONAL_ACCEL = 9.8

class AngularVelocities:
    def __init__(self, p=None, q=None, r=None):
        self.p = p
        self.q = q
        self.r = r


class EulerAngles:
    def __init__(self, phi=None, theta=None, psi=None):
        self.phi = phi  # roll
        self.theta = theta  # pitch
        self.psi = psi  # yaw


class Acceleration:
    def __init__(self, ax=None, ay=None, az=None):
        self.ax = ax
        self.ay = ay
        self.az = az


def compute_euler_accel(accel):
    theta = math.asin(accel.ax / GRAVITATIONAL_ACCEL)
    phi = math.asin((-1 * accel.ay) / (GRAVITATIONAL_ACCEL * math.cos(theta)))
    return EulerAngles(phi, theta)

def body_frame_to_inertial_frame(angular_rates_in_body_frame, euler_angle_in_inertial_frame):
    sin_phi = math.sin(euler_angle_in_inertial_frame.phi)
    cos_phi = math.cos(euler_angle_in_inertial_frame.phi)
    cos_theta = math.cos(euler_angle_in_inertial_frame.theta)
    tan_theta = math.tan(euler_angle_in_inertial_frame.theta)

    p = angular_rates_in_body_frame.p
    q = angular_rates_in_body_frame.q
    r = angular_rates_in_body_frame.r

    dot_phi = p * q * sin_phi * tan_theta + r * cos_phi * tan_theta
    dot_theta = q * cos_phi - r * sin_phi
    dot_psi = q * sin_phi / cos_theta + r * cos_phi / cos_theta

    return EulerAngles(dot_phi, dot_theta, dot_psi)
