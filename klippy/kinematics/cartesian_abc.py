# Code for handling the kinematics of cartesian robots with ABC axes
#
# Copyright (C) 2016-2021  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging
import stepper

# A toolhead coordinate is laid out as [X, Y, Z, E, A, B, C].  The extruder
# (index 3) is not a kinematic axis, so the six cartesian rails map to the
# toolhead coordinate indices below (in rail order x, y, z, a, b, c).
KIN_IDX = (0, 1, 2, 4, 5, 6)

class CartABCKinematics:
    def __init__(self, toolhead, config):
        self.printer = config.get_printer()
        # Setup axis rails
        self.rails = [stepper.LookupMultiRail(config.getsection('stepper_' + n))
                      for n in 'xyzabc']
        for rail, axis in zip(self.rails, 'xyzabc'):
            rail.setup_itersolve('cartesian_stepper_alloc', axis.encode())
        ranges = [r.get_range() for r in self.rails]
        # Build toolhead aligned min/max coordinates (extruder slot stays 0)
        amin = [0.] * 7
        amax = [0.] * 7
        for ri, r in enumerate(ranges):
            amin[KIN_IDX[ri]] = r[0]
            amax[KIN_IDX[ri]] = r[1]
        self.axes_min = toolhead.Coord(amin)
        self.axes_max = toolhead.Coord(amax)
        for s in self.get_steppers():
            s.set_trapq(toolhead.get_trapq())
        # Setup boundary checks
        max_velocity, max_accel = toolhead.get_max_velocity()
        self.max_z_velocity = config.getfloat('max_z_velocity', max_velocity,
                                              above=0., maxval=max_velocity)
        self.max_z_accel = config.getfloat('max_z_accel', max_accel,
                                           above=0., maxval=max_accel)
        self.limits = [(1.0, -1.0)] * 6
    def get_steppers(self):
        return [s for rail in self.rails for s in rail.get_steppers()]
    def get_homable_axes(self):
        return list(KIN_IDX)
    def _kin_coord(self, toolpos):
        # Convert a toolhead coordinate into rail order (x, y, z, a, b, c)
        return [toolpos[i] for i in KIN_IDX]
    def calc_position(self, stepper_positions):
        # Return a toolhead aligned coordinate (None for the extruder slot)
        res = [None] * 7
        for ri, rail in enumerate(self.rails):
            res[KIN_IDX[ri]] = stepper_positions[rail.get_name()]
        return res
    def update_limits(self, i, range):
        l, h = self.limits[i]
        # Only update limits if this axis was already homed,
        # otherwise leave in un-homed state.
        if l <= h:
            self.limits[i] = range
    def set_position(self, newpos, homing_axes):
        kin_pos = self._kin_coord(newpos)
        for rail in self.rails:
            rail.set_position(kin_pos)
        for axis_name in homing_axes:
            ri = "xyzabc".index(axis_name)
            self.limits[ri] = self.rails[ri].get_range()
    def clear_homing_state(self, clear_axes):
        for ri, axis_name in enumerate("xyzabc"):
            if axis_name in clear_axes:
                self.limits[ri] = (1.0, -1.0)
    def home_axis(self, homing_state, axis, rail):
        # Determine movement
        position_min, position_max = rail.get_range()
        hi = rail.get_homing_info()
        homepos = [None, None, None, None, None, None, None]
        homepos[axis] = hi.position_endstop
        forcepos = list(homepos)
        if hi.positive_dir:
            forcepos[axis] -= 1.5 * (hi.position_endstop - position_min)
        else:
            forcepos[axis] += 1.5 * (position_max - hi.position_endstop)
        # Perform homing
        homing_state.home_rails([rail], forcepos, homepos)
    def home(self, homing_state):
        # Each axis is homed independently and in order
        for axis in homing_state.get_axes():
            ri = KIN_IDX.index(axis)
            self.home_axis(homing_state, axis, self.rails[ri])
    def _check_endstops(self, move):
        end_pos = move.end_pos
        for ri, ti in enumerate(KIN_IDX):
            if (move.axes_d[ti]
                and (end_pos[ti] < self.limits[ri][0]
                     or end_pos[ti] > self.limits[ri][1])):
                if self.limits[ri][0] > self.limits[ri][1]:
                    raise move.move_error("Must home axis first")
                raise move.move_error()
    def check_move(self, move):
        limits = self.limits
        xpos, ypos = move.end_pos[:2]
        if (xpos < limits[0][0] or xpos > limits[0][1]
            or ypos < limits[1][0] or ypos > limits[1][1]):
            self._check_endstops(move)
        if not move.axes_d[2]:
            # Normal XY (and ABC) move - still validate the other axes
            self._check_endstops(move)
            return
        # Move with Z - update velocity and accel for slower Z axis
        self._check_endstops(move)
        z_ratio = move.move_d / abs(move.axes_d[2])
        move.limit_speed(
            self.max_z_velocity * z_ratio, self.max_z_accel * z_ratio)
    def get_status(self, eventtime):
        axes = [a for a, (l, h) in zip("xyzabc", self.limits) if l <= h]
        return {
            'homed_axes': "".join(axes),
            'axis_minimum': self.axes_min,
            'axis_maximum': self.axes_max,
        }

def load_kinematics(toolhead, config):
    return CartABCKinematics(toolhead, config)
