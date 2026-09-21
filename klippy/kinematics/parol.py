# Kinematics for the Source Robotics PAROL6 joint-space controller.
#
# Copyright (C) 2026
# This file may be distributed under the terms of the GNU GPLv3 license.

"""PAROL6 six-axis, joint-space kinematics.

The Klipper coordinates X, Y, Z, A, B, and C represent joints J1 through J6
respectively, in degrees.  This is intentionally a joint-space kinematics:
Klipper plans the individual joint trajectory, while Cartesian/inverse-
kinematics conversion belongs in the host application that produces G-code.
"""

import stepper


# Toolhead coordinate indexes for X, Y, Z, A, B, C.  E remains index 3.
KIN_IDX = (0, 1, 2, 4, 5, 6)
AXES = "xyzabc"


class ParolKinematics:
    def __init__(self, toolhead, config):
        self.printer = config.get_printer()
        self.toolhead = toolhead
        self.rails = [stepper.LookupMultiRail(config.getsection('stepper_' + n))
                      for n in AXES]
        for rail, axis in zip(self.rails, AXES):
            rail.setup_itersolve('cartesian_stepper_alloc', axis.encode())
        ranges = [rail.get_range() for rail in self.rails]
        amin = [0.] * 7
        amax = [0.] * 7
        for ri, (low, high) in enumerate(ranges):
            amin[KIN_IDX[ri]] = low
            amax[KIN_IDX[ri]] = high
        self.axes_min = toolhead.Coord(amin)
        self.axes_max = toolhead.Coord(amax)
        for stepper_ in self.get_steppers():
            stepper_.set_trapq(toolhead.get_trapq())

        # A range is inverted until its joint has been homed.
        self.limits = [(1., -1.)] * 6

        # J6 must be parked before J5 can approach its switch.  The supplied
        # default is the documented standby angle; it may be overridden for a
        # custom gripper or switch geometry.
        self.j6_j5_homing_position = config.getfloat(
            'j6_j5_homing_position', 180.)
        j6_min, j6_max = ranges[5]
        if not j6_min <= self.j6_j5_homing_position <= j6_max:
            raise config.error("j6_j5_homing_position must be within J6 limits")

        # These are deliberately conservative defaults in degree units.
        self.max_z_velocity = config.getfloat(
            'max_z_velocity', toolhead.get_max_velocity()[0], above=0.)
        self.max_z_accel = config.getfloat(
            'max_z_accel', toolhead.get_max_velocity()[1], above=0.)

    def get_steppers(self):
        return [s for rail in self.rails for s in rail.get_steppers()]

    def get_homable_axes(self):
        return list(KIN_IDX)

    def _kin_coord(self, toolpos):
        return [toolpos[i] for i in KIN_IDX]

    def calc_position(self, stepper_positions):
        result = [None] * 7
        for ri, rail in enumerate(self.rails):
            result[KIN_IDX[ri]] = stepper_positions[rail.get_name()]
        return result

    def set_position(self, newpos, homing_axes):
        kin_pos = self._kin_coord(newpos)
        for rail in self.rails:
            rail.set_position(kin_pos)
        for axis in homing_axes:
            ri = AXES.index(axis)
            self.limits[ri] = self.rails[ri].get_range()

    def clear_homing_state(self, clear_axes):
        for ri, axis in enumerate(AXES):
            if axis in clear_axes:
                self.limits[ri] = (1., -1.)

    def _home_rails(self, homing_state, rail_indexes):
        """Home one or more independent PAROL joints in one operation."""
        homepos = [None] * 7
        forcepos = [None] * 7
        rails = []
        for ri in rail_indexes:
            rail = self.rails[ri]
            axis = KIN_IDX[ri]
            position_min, position_max = rail.get_range()
            hi = rail.get_homing_info()
            homepos[axis] = hi.position_endstop
            forcepos[axis] = hi.position_endstop
            if hi.positive_dir:
                forcepos[axis] -= 1.5 * (hi.position_endstop - position_min)
            else:
                forcepos[axis] += 1.5 * (position_max - hi.position_endstop)
            rails.append(rail)
        homing_state.home_rails(rails, forcepos, homepos)

    def _move_joints(self, positions):
        """Move already-homed joints without changing any other coordinate."""
        target = list(self.toolhead.get_position())
        for ri, position in positions.items():
            target[KIN_IDX[ri]] = position
        self.toolhead.move(target, self.toolhead.get_max_velocity()[0])

    def home(self, homing_state):
        # Official PAROL sequence:
        # 1) J1/J2/J3 together, 2) J4, 3) J6, park J6, 4) J5, then park J5/J6.
        # If G28 requested only some axes, preserve that subset and do not make
        # clearance/standby moves that require an omitted axis to be homed.
        requested = {KIN_IDX.index(axis) for axis in homing_state.get_axes()}
        first_group = [ri for ri in (0, 1, 2) if ri in requested]
        if first_group:
            self._home_rails(homing_state, first_group)
            if len(first_group) == 3:
                self._move_joints({0: 0., 1: -90., 2: 180.})
        if 3 in requested:
            self._home_rails(homing_state, [3])
            self._move_joints({3: 0.})
        if 5 in requested:
            self._home_rails(homing_state, [5])
            if 4 in requested:
                self._move_joints({5: self.j6_j5_homing_position})
        if 4 in requested:
            self._home_rails(homing_state, [4])
            if 5 in requested:
                self._move_joints({4: 0., 5: 180.})

    def _check_endstops(self, move):
        for ri, ti in enumerate(KIN_IDX):
            if (move.axes_d[ti]
                    and (move.end_pos[ti] < self.limits[ri][0]
                         or move.end_pos[ti] > self.limits[ri][1])):
                if self.limits[ri][0] > self.limits[ri][1]:
                    raise move.move_error("Must home axis first")
                raise move.move_error()

    def check_move(self, move):
        self._check_endstops(move)
        # J3 is normally the most heavily loaded PAROL joint.  Keep Klipper's
        # usual max_z_* mechanism, with Z interpreted as J3 degrees here.
        if move.axes_d[2]:
            ratio = move.move_d / abs(move.axes_d[2])
            move.limit_speed(self.max_z_velocity * ratio,
                             self.max_z_accel * ratio)

    def get_status(self, eventtime):
        axes = [axis for axis, (low, high) in zip(AXES, self.limits)
                if low <= high]
        return {'homed_axes': ''.join(axes),
                'axis_minimum': self.axes_min,
                'axis_maximum': self.axes_max}


def load_kinematics(toolhead, config):
    return ParolKinematics(toolhead, config)
