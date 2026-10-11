# Code for simple linear motor support (A, B, C axes)
#
# Copyright (C) 2025  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

class LinearMotor:
    """A simple linear motor for non-kinematic axes (A, B, C)."""
    
    def __init__(self, stepper, name):
        self.stepper = stepper
        self.name = name
        self.printer = stepper.printer
        self.trapq = stepper.trapq
        self.trapq_append = stepper.trapq_append
        self.commanded_pos = 0.
        self.last_position = 0.
        self.axis_gcode_id = None
        self.instant_corner_v = 1.
        self.gaxis_limit_velocity = 999999.9
        self.gaxis_limit_accel = 999999.9
    
    def check_move(self, move, ea_index):
        """Check move limits for this axis."""
        movepos = move.end_pos[ea_index]
        
        # Check stepper limits if configured
        if hasattr(self.stepper, 'limit_minimum') and hasattr(self.stepper, 'limit_maximum'):
            if ((self.stepper.limit_minimum is not None and movepos < self.stepper.limit_minimum)
                or (self.stepper.limit_maximum is not None and movepos > self.stepper.limit_maximum)):
                raise move.move_error()
        
        # Check if need to limit maximum velocity and acceleration
        if move.axes_d[ea_index]:
            axis_ratio = move.move_d / abs(move.axes_d[ea_index])
            limit_velocity = self.gaxis_limit_velocity * axis_ratio
            limit_accel = self.gaxis_limit_accel * axis_ratio
            if not move.is_kinematic_move and hasattr(self.stepper, 'accel'):
                if self.stepper.accel:
                    limit_accel = min(limit_accel, self.stepper.accel * axis_ratio)
            move.limit_speed(limit_velocity, limit_accel)
    
    def process_move(self, print_time, move, ea_index):
        """Process a move for this linear motor."""
        axis_r = move.axes_r[ea_index]
        start_pos = move.start_pos[ea_index]
        accel = move.accel * axis_r
        start_v = move.start_v * axis_r
        cruise_v = move.cruise_v * axis_r
        
        # Queue movement for this single axis
        # Format: x position is the axis position, y/z/a/b/c are unused (0)
        self.trapq_append(self.trapq, print_time,
                          move.accel_t, move.cruise_t, move.decel_t,
                          start_pos, 0., 0., 0., 0., 0.,
                          1., 0., 0., 0., 0., 0.,
                          start_v, cruise_v, accel)
        self.last_position = move.end_pos[ea_index]
    
    def find_past_position(self, print_time):
        """Return the position at a given print time."""
        return self.stepper.find_past_position(print_time)
    
    def calc_junction(self, prev_move, move, ea_index):
        """Calculate junction velocity limit."""
        diff_r = move.axes_r[ea_index] - prev_move.axes_r[ea_index]
        if diff_r:
            return (self.instant_corner_v / abs(diff_r))**2
        return move.max_cruise_v2
    
    def get_name(self):
        """Return the axis name."""
        return self.name
    
    def get_axis_gcode_id(self):
        """Return the gcode ID for this axis (A, B, C, etc)."""
        return self.axis_gcode_id
    
    def get_trapq(self):
        """Return the trapq object for this axis."""
        return self.trapq
    
    def command_with_gcode_axis(self, gcmd):
        """Register/unregister this axis with the gcode interface."""
        toolhead = self.printer.lookup_object('toolhead')
        gcode_axis = gcmd.get('GCODE_AXIS').upper()
        instant_corner_v = gcmd.get_float('INSTANTANEOUS_CORNER_VELOCITY', 1.,
                                          minval=0.)
        limit_velocity = gcmd.get_float('LIMIT_VELOCITY', 999999.9, above=0.)
        limit_accel = gcmd.get_float('LIMIT_ACCEL', 999999.9, above=0.)
        
        if self.axis_gcode_id is not None:
            if gcode_axis:
                raise gcmd.error("Must unregister axis first")
            # Unregister
            toolhead.remove_extra_axis(self)
            self.axis_gcode_id = None
            return
        
        if (len(gcode_axis) != 1 or not gcode_axis.isupper()
            or gcode_axis in "XYZEFN"):
            if not gcode_axis:
                # Request to unregister already unregistered axis
                return
            raise gcmd.error("Not a valid GCODE_AXIS")
        
        for ea in toolhead.get_extra_axes():
            if ea is not None and ea.get_axis_gcode_id() == gcode_axis:
                raise gcmd.error("Axis '%s' already registered" % (gcode_axis,))
        
        self.axis_gcode_id = gcode_axis
        self.instant_corner_v = instant_corner_v
        self.gaxis_limit_velocity = limit_velocity
        self.gaxis_limit_accel = limit_accel
        toolhead.add_extra_axis(self, self.commanded_pos)
