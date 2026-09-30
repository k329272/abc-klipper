# Cartesian endpoint inverse kinematics for the PAROL6 joint-space kinematics.
#
# Copyright (C) 2026
# This file may be distributed under the terms of the GNU GPLv3 license.

"""PAROL6 Cartesian endpoint IK.

Use PAROL_MOVE rather than G1 for Cartesian targets.  The underlying PAROL
kinematics remains joint-space so G28 can safely home one joint at a time.
PAROL_MOVE solves a target pose and queues the resulting synchronized joint
move.  It is deliberately an endpoint planner; applications needing a straight
TCP path must interpolate poses into several PAROL_MOVE commands.
"""

import math
import chelper


RAD = math.pi / 180.


def _matmul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(4)) for j in range(4)]
            for i in range(4)]


def _dh(theta, alpha, r, d):
    ct, st, ca, sa = (
        math.cos(theta),
        math.sin(theta),
        math.cos(alpha),
        math.sin(alpha),
    )
    return [[ct, -st * ca, st * sa, r * ct],
            [st, ct * ca, -ct * sa, r * st],
            [0., sa, ca, d], [0., 0., 0., 1.]]


def _rpy(roll, pitch, yaw):
    # Fixed-frame Z-Y-X yaw/pitch/roll, all arguments in radians.
    cr, sr, cp, sp, cy, sy = (math.cos(roll), math.sin(roll), math.cos(pitch),
                              math.sin(pitch), math.cos(yaw), math.sin(yaw))
    return [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr, 0.],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr, 0.],
            [-sp, cp * sr, cp * cr, 0.], [0., 0., 0., 1.]]


def _rotvec(target, current):
    # Small-angle orientation error for target * current.transpose().
    r = [[sum(target[i][k] * current[j][k] for k in range(3))
          for j in range(3)] for i in range(3)]
    return [.5 * (r[2][1] - r[1][2]), .5 * (r[0][2] - r[2][0]),
            .5 * (r[1][0] - r[0][1])]


class ParolIK:
    def __init__(self, config):
        self.printer = config.get_printer()
        kin = self.printer.lookup_object('toolhead').get_kinematics()
        if kin.__class__.__module__ != 'kinematics.parol':
            raise config.error(
                "[parol_ik] requires [printer] kinematics: parol")
        self.kin = kin
        self.a1 = config.getfloat('a1', 110.50)
        self.a2 = config.getfloat('a2', 23.42)
        self.a3 = config.getfloat('a3', 180.00)
        self.a4 = config.getfloat('a4', 43.50)
        self.a5 = config.getfloat('a5', 176.35)
        self.a6 = config.getfloat('a6', 62.80)
        self.a7 = config.getfloat('a7', 45.25)
        # TCP/extruder offset from the standard PAROL flange/gripper frame.
        self.tool_offset = [config.getfloat('tool_offset_x', 0.),
                            config.getfloat('tool_offset_y', 0.),
                            config.getfloat('tool_offset_z', 0.)]
        self.orientation_scale = config.getfloat(
            'orientation_scale', 100., above=0.)
        self.max_iterations = config.getint('max_iterations', 80, minval=1)
        self.tolerance = config.getfloat('tolerance', .02, above=0.)
        ffi_main, ffi_lib = chelper.get_ffi()
        self.ffi_main = ffi_main
        self.ffi_lib = ffi_lib
        self.c_ik = ffi_main.gc(ffi_lib.parol_ik_alloc(), ffi_lib.free)
        ffi_lib.parol_ik_set_params(
            self.c_ik, self.a1, self.a2, self.a3, self.a4, self.a5,
            self.a6, self.a7, *self.tool_offset, self.orientation_scale)
        self.gcode = self.printer.lookup_object('gcode')
        self.gcode.register_command('PAROL_IK', self.cmd_PAROL_IK,
                                    desc='Report PAROL joint solution'
                                         ' for a TCP pose')
        self.gcode.register_command('PAROL_MOVE', self.cmd_PAROL_MOVE,
                                    desc='Move PAROL TCP to a Cartesian'
                                         ' endpoint')

    def fk(self, q):
        q = [v * RAD for v in q]
        table = ((q[0], -math.pi / 2., self.a2, self.a1),
                 (q[1] - math.pi / 2., math.pi, self.a3, 0.),
                 (q[2] + math.pi, math.pi / 2., -self.a4, 0.),
                 (q[3], -math.pi / 2., 0., -self.a5),
                 (q[4], math.pi / 2., 0., 0.),
                 (q[5] + math.pi, math.pi, -self.a7, -self.a6))
        t = [[1., 0., 0., 0.],
             [0., 1., 0., 0.],
             [0., 0., 1., 0.],
             [0., 0., 0., 1.]]
        for row in table:
            t = _matmul(t, _dh(*row))
        offset = [[1., 0., 0., self.tool_offset[0]],
                  [0., 1., 0., self.tool_offset[1]],
                  [0., 0., 1., self.tool_offset[2]], [0., 0., 0., 1.]]
        return _matmul(t, offset)

    def _target(self, gcmd):
        # RX/RY/RZ avoid ambiguity with joint-space A/B/C words used by G1.
        target = _rpy(gcmd.get_float('RX') * RAD, gcmd.get_float('RY') * RAD,
                      gcmd.get_float('RZ') * RAD)
        for index, axis in enumerate(('X', 'Y', 'Z')):
            target[index][3] = gcmd.get_float(axis)
        return target

    def _error(self, target, q):
        current = self.fk(q)
        pos = [target[i][3] - current[i][3] for i in range(3)]
        rot = _rotvec(target, current)
        return pos + [v * self.orientation_scale for v in rot]

    @staticmethod
    def _solve(a, b):
        # Gaussian elimination with partial pivoting; returns None if singular.
        a = [row[:] + [value] for row, value in zip(a, b)]
        for col in range(6):
            pivot = max(range(col, 6), key=lambda row: abs(a[row][col]))
            if abs(a[pivot][col]) < 1.e-12:
                return None
            a[col], a[pivot] = a[pivot], a[col]
            scale = a[col][col]
            a[col] = [v / scale for v in a[col]]
            for row in range(6):
                if row == col:
                    continue
                scale = a[row][col]
                a[row] = [v - scale * w for v, w in zip(a[row], a[col])]
        return [a[i][6] for i in range(6)]

    def ik(self, target, seed):
        # The compiled chelper performs the FK/Jacobian/DLS iterations.  Keep
        # this wrapper intentionally thin so G-code processing stays Python.
        target_c = self.ffi_main.new(
            'double[12]', [target[i][j] for i in range(3) for j in range(3)]
            + [target[i][3] for i in range(3)])
        seed_c = self.ffi_main.new('double[6]', seed)
        result_c = self.ffi_main.new('double[6]')
        if not self.ffi_lib.parol_ik_solve(
                self.c_ik, target_c, seed_c, self.max_iterations,
                self.tolerance, result_c):
            return list(result_c)
        raise self.printer.command_error(
            'PAROL IK failed to converge;'
            ' target may be unreachable or singular')

    def _python_ik(self, target, seed):
        q = list(seed)
        delta = .02  # degrees; numerical Jacobian step
        for unused in range(self.max_iterations):
            error = self._error(target, q)
            if max(abs(v) for v in error) <= self.tolerance:
                return q
            jacobian = [[0.] * 6 for unused in range(6)]
            for col in range(6):
                q2 = q[:]
                q2[col] += delta
                e2 = self._error(target, q2)
                for row in range(6):
                    jacobian[row][col] = -(e2[row] - error[row]) / delta
            # Damped least-squares: (J'J + lambda^2 I) dq = J' error.
            normal = [[sum(jacobian[k][i] * jacobian[k][j] for k in range(6))
                       + (.01 if i == j else 0.) for j in range(6)]
                      for i in range(6)]
            rhs = [sum(jacobian[k][i] * error[k] for k in range(6))
                   for i in range(6)]
            dq = self._solve(normal, rhs)
            if dq is None:
                break
            q = [angle + max(-5., min(5., change))
                 for angle, change in zip(q, dq)]
        raise self.printer.command_error('PAROL IK failed to converge;'
                                         'target is be unreachable or singular')

    def _solution(self, gcmd):
        toolhead = self.printer.lookup_object('toolhead')
        current = toolhead.get_position()
        seed = [current[i] for i in (0, 1, 2, 4, 5, 6)]
        q = self.ik(self._target(gcmd), seed)
        for angle, rail in zip(q, self.kin.rails):
            low, high = rail.get_range()
            if not low <= angle <= high:
                raise self.printer.command_error(
                    'PAROL IK solution exceeds %s range:'
                    '%.4f not in %.4f..%.4f'
                    % (rail.get_name(), angle, low, high))
        return q

    def cmd_PAROL_IK(self, gcmd):
        q = self._solution(gcmd)
        gcmd.respond_info('PAROL IK: J1=%.5f J2=%.5f J3=%.5f'
                          'J4=%.5f J5=%.5fJ6=%.5f'
                          % tuple(q))

    def cmd_PAROL_MOVE(self, gcmd):
        q = self._solution(gcmd)
        speed = gcmd.get_float('VELOCITY', None, above=0.)
        if speed is None:
            speed = gcmd.get_float('F', None, above=0.)
            if speed is not None:
                speed /= 60.
        if speed is None:
            speed = self.printer.lookup_object('toolhead').get_max_velocity()[0]
        target = list(self.printer.lookup_object('toolhead').get_position())
        for index, angle in zip((0, 1, 2, 4, 5, 6), q):
            target[index] = angle
        self.printer.lookup_object('toolhead').move(target, speed)


def load_config(config):
    return ParolIK(config)
