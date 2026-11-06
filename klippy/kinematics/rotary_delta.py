# Code for handling the kinematics of rotary delta robots
#
# Copyright (C) 2019-2021  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import math, logging
import stepper, mathutil


class RotaryDeltaKinematics:
    def __init__(self, toolhead, config):
        # Setup the three or six rotary delta arm rails.
        axes = 'abc'
        if any(config.has_section('stepper_' + a) for a in 'def'):
            if not all(config.has_section('stepper_' + a) for a in 'def'):
                raise config.get_printer().config_error(
                    "Rotary delta kinematics requires stepper_d, stepper_e, "
                    "and stepper_f together")
            axes = 'abcdef'
        stepper_configs = [config.getsection('stepper_' + a) for a in axes]
        a_endstop = stepper_configs[0].getfloat('position_endstop')
        self.rails = [stepper.LookupRail(
            sconfig, need_position_minmax=False,
            default_position_endstop=a_endstop, units_in_radians=True)
            for sconfig in stepper_configs]

        self.bed = None
        if config.has_section('stepper_bed'):
            self.bed = stepper.PrinterStepper(
                config.getsection('stepper_bed'), units_in_radians=True)
            self.bed.setup_itersolve('rotary_delta_bed_stepper_alloc')

        max_velocity, max_accel = toolhead.get_max_velocity()
        self.max_velocity = max_velocity
        self.max_accel = max_accel
        self.max_z_velocity = config.getfloat(
            'max_z_velocity', max_velocity, above=0., maxval=max_velocity)
        self.max_z_accel = config.getfloat(
            'max_z_accel', max_accel, above=0., maxval=max_accel)
        self.max_c_velocity = config.getfloat(
            'max_c_velocity', max_velocity, above=0.)
        self.max_c_accel = config.getfloat(
            'max_c_accel', max_accel, above=0.)

        shoulder_radius = config.getfloat('shoulder_radius', above=0.)
        shoulder_height = config.getfloat('shoulder_height', above=0.)
        a_upper_arm = stepper_configs[0].getfloat(
            'upper_arm_length', above=0.)
        upper_arms = [sconfig.getfloat('upper_arm_length', a_upper_arm,
                                      above=0.)
                      for sconfig in stepper_configs]
        a_lower_arm = stepper_configs[0].getfloat(
            'lower_arm_length', above=0.)
        lower_arms = [sconfig.getfloat('lower_arm_length', a_lower_arm,
                                      above=0.)
                      for sconfig in stepper_configs]
        angles = [sconfig.getfloat('angle', default_angle)
                  for sconfig, default_angle in zip(
                      stepper_configs, [30., 150., 270., 90., 210., 330.])]
        endstops = [rail.get_homing_info().position_endstop
                    for rail in self.rails]
        stepdists = [rail.get_steppers()[0].get_step_dist()
                     for rail in self.rails]

        self.calibration = RotaryDeltaCalibration(
            shoulder_radius, shoulder_height, angles, upper_arms, lower_arms,
            endstops, stepdists)
        for rail, angle, upper_arm, lower_arm in zip(
                self.rails, angles, upper_arms, lower_arms):
            rail.setup_itersolve(
                'rotary_delta_stepper_alloc', shoulder_radius, shoulder_height,
                math.radians(angle), upper_arm, lower_arm)

        for s in self.get_steppers():
            s.set_trapq(toolhead.get_trapq())

        self.need_home = True
        self.limit_xy2 = -1.
        endstop_angles = [self.calibration.calc_arm_position(
            i, (0., 0., ep), 0.) for i, ep in enumerate(endstops)]
        self.home_position = tuple(
            self.calibration.actuator_to_cartesian(endstop_angles))
        self.max_z = min(endstops)
        self.min_z = config.getfloat('minimum_z_position',
                     0.,
                     maxval=self.max_z)
        min_ua = min(shoulder_radius + ua for ua in upper_arms)
        min_la = min(la - shoulder_radius for la in lower_arms)
        self.max_xy2 = min(min_ua, min_la)**2
        arm_z = [self.calibration.elbow_coord(i, ea)[2]
                  for i, ea in enumerate(endstop_angles)]
        self.limit_z = min(az - la for az, la in zip(arm_z, lower_arms))
        logging.info("Rotary delta max build height %.2fmm"
                     % (self.max_z,))
        max_xy = math.sqrt(self.max_xy2)
        self.axes_min = toolhead.Coord((-max_xy, -max_xy, self.min_z,
                                        0., 0., 0., 0.))
        self.axes_max = toolhead.Coord((max_xy, max_xy, self.max_z,
                                        0., 0., 0., 0.))
        self.set_position([0., 0., 0.], "")

    def get_steppers(self):
        steppers = [s for rail in self.rails for s in rail.get_steppers()]
        if self.bed is not None:
            steppers.append(self.bed)
        return steppers

    def calc_position(self, stepper_positions):
        spos = [stepper_positions[rail.get_name()] for rail in self.rails]
        xyz = self.calibration.actuator_to_cartesian(spos)
        result = [xyz[0], xyz[1], xyz[2], None, None, None, None]
        if self.bed is not None:
            result[6] = math.degrees(stepper_positions[self.bed.get_name()])
            angle = math.radians(result[6])
            c, s = math.cos(angle), math.sin(angle)
            # Report XY in the rotating bed coordinate frame.
            result[0], result[1] = (c * xyz[0] + s * xyz[1],
                                   -s * xyz[0] + c * xyz[1])
        return result

    def set_position(self, newpos, homing_axes):
        for rail in self.rails:
            rail.set_position(newpos)
        if self.bed is not None:
            self.bed.set_position(newpos)
        self.limit_xy2 = -1.
        if 'xyz' in homing_axes:
            self.need_home = False

    def clear_homing_state(self, clear_axes):
        if clear_axes:
            self.limit_xy2 = -1.
            self.need_home = True

    def home(self, homing_state):
        # All delta arms home simultaneously. The optional bed axis has no
        # endstop and is referenced to its configured startup position.
        homing_state.set_axes([0, 1, 2])
        forcepos = list(self.home_position)
        forcepos[2] = -1.
        homing_state.home_rails(self.rails, forcepos, self.home_position)

    def check_move(self, move):
        if move.axes_d[4] or move.axes_d[5]:
            raise move.move_error("Rotary delta does not support A or B moves")
        end_pos = move.end_pos
        end_xy2 = end_pos[0]**2 + end_pos[1]**2
        if (end_xy2 <= self.limit_xy2 and not move.axes_d[2]
                and not move.axes_d[6]):
            return
        if self.need_home:
            raise move.move_error("Must home first")
        end_z = end_pos[2]
        limit_xy2 = self.max_xy2
        if end_z > self.limit_z:
            limit_xy2 = min(limit_xy2, (self.max_z - end_z)**2)
        if (end_xy2 > limit_xy2 or end_z > self.max_z or end_z < self.min_z):
            if (end_pos[:2] != self.home_position[:2]
                    or end_z < self.min_z or end_z > self.home_position[2]):
                raise move.move_error()
            limit_xy2 = -1.
        if move.axes_d[2]:
            ratio = move.move_d / abs(move.axes_d[2])
            move.limit_speed(self.max_z_velocity * ratio,
                             self.max_z_accel * ratio)
            limit_xy2 = -1.
        if self.bed is not None and move.axes_d[6]:
            ratio = move.move_d / abs(move.axes_d[6])
            move.limit_speed(self.max_c_velocity * ratio,
                             self.max_c_accel * ratio)
            limit_xy2 = -1.
        self.limit_xy2 = limit_xy2

    def get_status(self, eventtime):
        return {
            'homed_axes': '' if self.need_home else 'xyz',
            'axis_minimum': self.axes_min,
            'axis_maximum': self.axes_max,
        }

    def get_calibration(self):
        return self.calibration


# Rotary delta parameter calibration for DELTA_CALIBRATE.
class RotaryDeltaCalibration:
    def __init__(self, shoulder_radius, shoulder_height, angles,
                 upper_arms, lower_arms, endstops, stepdists):
        self.shoulder_radius = shoulder_radius
        self.shoulder_height = shoulder_height
        self.angles = angles
        self.upper_arms = upper_arms
        self.lower_arms = lower_arms
        self.endstops = endstops
        self.stepdists = stepdists
        self.abs_endstops = [self.calc_arm_position(i, (0., 0., es), 0.)
                             for i, es in enumerate(endstops)]

    def coordinate_descent_params(self, is_extended):
        axes = 'abcdef'[:len(self.angles)]
        adj_params = (('shoulder_height',)
                      + tuple('endstop_'+axis for axis in axes))
        if is_extended:
            adj_params += (('shoulder_radius', 'angle_a', 'angle_b')
                           + tuple('angle_'+axis for axis in axes[3:]))
        params = {'shoulder_radius': self.shoulder_radius,
                  'shoulder_height': self.shoulder_height}
        for i, axis in enumerate(axes):
            params['angle_'+axis] = self.angles[i]
            params['upper_arm_'+axis] = self.upper_arms[i]
            params['lower_arm_'+axis] = self.lower_arms[i]
            params['endstop_'+axis] = self.endstops[i]
            params['stepdist_'+axis] = self.stepdists[i]
        return adj_params, params

    def new_calibration(self, params):
        shoulder_radius = params['shoulder_radius']
        shoulder_height = params['shoulder_height']
        axes = 'abcdef'[:len(self.angles)]
        angles = [params['angle_'+a] for a in axes]
        upper_arms = [params['upper_arm_'+a] for a in axes]
        lower_arms = [params['lower_arm_'+a] for a in axes]
        endstops = [params['endstop_'+a] for a in axes]
        stepdists = [params['stepdist_'+a] for a in axes]
        return RotaryDeltaCalibration(
            shoulder_radius, shoulder_height, angles, upper_arms, lower_arms,
            endstops, stepdists)

    def elbow_coord(self, elbow_id, spos):
        sj_elbow_x = self.upper_arms[elbow_id] * math.cos(spos)
        sj_elbow_y = self.upper_arms[elbow_id] * math.sin(spos)
        angle = math.radians(self.angles[elbow_id])
        x = (sj_elbow_x + self.shoulder_radius) * math.cos(angle)
        y = (sj_elbow_x + self.shoulder_radius) * math.sin(angle)
        z = sj_elbow_y + self.shoulder_height
        return (x, y, z)

    def actuator_to_cartesian(self, spos):
        sphere_coords = [self.elbow_coord(i, sp) for i, sp in enumerate(spos)]
        return mathutil.trilateration(
            sphere_coords, [la**2 for la in self.lower_arms])

    def calc_arm_position(self, arm_id, coord, bed_angle=0.):
        # Transform bed-local XY coordinates into the fixed printer frame.
        angle = math.radians(bed_angle)
        x = coord[0] * math.cos(angle) - coord[1] * math.sin(angle)
        y = coord[0] * math.sin(angle) + coord[1] * math.cos(angle)
        shoulder_angle = math.radians(self.angles[arm_id])
        sjx = (x * math.cos(shoulder_angle) + y * math.sin(shoulder_angle)
               - self.shoulder_radius)
        sjz = y * math.cos(shoulder_angle) - x * math.sin(shoulder_angle)
        sjy = coord[2] - self.shoulder_height
        lower2 = self.lower_arms[arm_id]**2 - sjz**2
        distance2 = sjx**2 + sjy**2
        distance = math.sqrt(distance2)
        upper2 = self.upper_arms[arm_id]**2
        along = (upper2 + distance2 - lower2) / (2. * distance)
        height = math.sqrt(upper2 - along**2)
        elbow_x = along * sjx / distance + height * sjy / distance
        elbow_y = along * sjy / distance - height * sjx / distance
        return math.atan2(elbow_y, elbow_x)

    def get_position_from_stable(self, stable_position):
        spos = [ea - sp * sd for ea, sp, sd in zip(
            self.abs_endstops, stable_position, self.stepdists)]
        return self.actuator_to_cartesian(spos)

    def calc_stable_position(self, coord):
        bed_angle = coord[6] if len(coord) > 6 else 0.
        pos = [self.calc_arm_position(i, coord, bed_angle)
               for i in range(len(self.angles))]
        return [(ep - sp) / sd for sd, ep, sp in zip(
            self.stepdists, self.abs_endstops, pos)]

    def save_state(self, configfile):
        configfile.set('printer', 'shoulder_radius', "%.6f"
                       % (self.shoulder_radius,))
        configfile.set('printer', 'shoulder_height', "%.6f"
                       % (self.shoulder_height,))
        axes = 'abcdef'[:len(self.angles)]
        for i, axis in enumerate(axes):
            configfile.set('stepper_'+axis, 'angle', "%.6f" % (self.angles[i],))
            configfile.set('stepper_'+axis, 'position_endstop',
                           "%.6f" % (self.endstops[i],))
        gcode = configfile.get_printer().lookup_object("gcode")
        lines = ["stepper_%s: position_endstop: %.6f angle: %.6f"
                 % (axis, self.endstops[i], self.angles[i])
                 for i, axis in enumerate(axes)]
        lines.append("shoulder_radius: %.6f shoulder_height: %.6f"
                     % (self.shoulder_radius, self.shoulder_height))
        gcode.respond_info("\n".join(lines))


def load_kinematics(toolhead, config):
    return RotaryDeltaKinematics(toolhead, config)
