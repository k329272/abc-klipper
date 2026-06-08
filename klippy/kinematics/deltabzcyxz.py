import math, logging
import stepper, chelper

# Coordinate system: X, Y, Z (Cartesian), B (rotation around Y), C (rotation around Z)
# Toolhead offset is applied based on B angle

class DeltaBZCYXZKinematics:
    def __init__(self, toolhead, config):
        self.printer = config.get_printer()
        self.toolhead = toolhead
        # Read toolhead offset from config (default 0, 0)
        toolhead_offset_x = config.getfloat('toolhead_offset_x', 0., minval=-100., maxval=100.)
        toolhead_offset_y = config.getfloat('toolhead_offset_y', 0., minval=-100., maxval=100.)
        self.toolhead_offset_hypot = math.hypot(toolhead_offset_x, toolhead_offset_y)
        self.toolhead_offset_angle = math.atan2(toolhead_offset_y, toolhead_offset_x)
        # Setup axis rails for X, Y, Z, B, C using optimized C kinematics
        ffi_main, ffi_lib = chelper.get_ffi()
        self.rails = []
        for axis_char in 'xyzbc':
            rail = stepper.LookupMultiRail(config.getsection('stepper_' + axis_char))
            # Use C implementation for faster kinematics
            sk = ffi_main.gc(
                ffi_lib.deltabzcyxz_stepper_alloc(
                    ord(axis_char), self.toolhead_offset_hypot, self.toolhead_offset_angle),
                ffi_lib.deltabzcyxz_stepper_free)
            rail.get_steppers()[0].set_stepper_kinematics(sk)
            self.rails.append(rail)
        ranges = [r.get_range() for r in self.rails]
        # Build toolhead aligned min/max coordinates
        self.axes_min = toolhead.Coord([ranges[0][0], ranges[1][0], ranges[2][0],
                                        0., ranges[3][0], ranges[4][0], 0.])
        self.axes_max = toolhead.Coord([ranges[0][1], ranges[1][1], ranges[2][1],
                                        0., ranges[3][1], ranges[4][1], 0.])
        for s in self.get_steppers():
            s.set_trapq(toolhead.get_trapq())
        # Setup boundary checks
        max_velocity, max_accel = toolhead.get_max_velocity()
        self.max_z_velocity = config.getfloat('max_z_velocity', max_velocity,
                                              above=0., maxval=max_velocity)
        self.max_z_accel = config.getfloat('max_z_accel', max_accel,
                                           above=0., maxval=max_accel)
        self.limits = [(1.0, -1.0)] * 5
    
    def get_steppers(self):
        return [s for rail in self.rails for s in rail.get_steppers()]
    
    def get_homable_axes(self):
        return [0, 1, 2, 4, 5]  # X, Y, Z, B, C
    
    def calc_position(self, stepper_positions):
        """Return toolhead aligned coordinate from stepper positions"""
        res = [None] * 7
        rail_names = ['stepper_x', 'stepper_y', 'stepper_z', 'stepper_b', 'stepper_c']
        axes_indices = [0, 1, 2, 4, 5]
        for i, axis_idx in enumerate(axes_indices):
            if rail_names[i] in stepper_positions:
                res[axis_idx] = stepper_positions[rail_names[i]]
        return res
    
    def update_limits(self, i, range):
        """Update axis limits"""
        l, h = self.limits[i]
        if l <= h:
            self.limits[i] = range
    
    def set_position(self, newpos, homing_axes):
        """Set position after homing"""
        kin_pos = [newpos[i] for i in [0, 1, 2, 4, 5]]
        for i, rail in enumerate(self.rails):
            rail.set_position(kin_pos)
        for axis_name in homing_axes:
            if axis_name in 'xyzbc':
                ri = 'xyzbc'.index(axis_name)
                self.limits[ri] = self.rails[ri].get_range()
    
    def clear_homing_state(self, clear_axes):
        """Clear homing state for specified axes"""
        for ri, axis_name in enumerate('xyzbc'):
            if axis_name in clear_axes:
                self.limits[ri] = (1.0, -1.0)
    
    def home_axis(self, homing_state, axis, rail):
        """Home a single axis"""
        position_min, position_max = rail.get_range()
        hi = rail.get_homing_info()
        homepos = [None] * 7
        homepos[axis] = hi.position_endstop
        forcepos = list(homepos)
        if hi.positive_dir:
            forcepos[axis] -= 1.5 * (hi.position_endstop - position_min)
        else:
            forcepos[axis] += 1.5 * (position_max - hi.position_endstop)
        homing_state.home_rails([rail], forcepos, homepos)
    
    def home(self, homing_state):
        """Home all requested axes in order"""
        axes_map = {'x': 0, 'y': 1, 'z': 2, 'b': 4, 'c': 5}
        for axis_char in 'xyzbc':
            axis_idx = axes_map[axis_char]
            if axis_idx in homing_state.get_axes():
                ri = 'xyzbc'.index(axis_char)
                self.home_axis(homing_state, axis_idx, self.rails[ri])
    
    def _check_endstops(self, move):
        """Check if move is within axis limits"""
        end_pos = move.end_pos
        axis_indices = [0, 1, 2, 4, 5]
        for ri, axis_idx in enumerate(axis_indices):
            if (move.axes_d[axis_idx]
                and (end_pos[axis_idx] < self.limits[ri][0]
                     or end_pos[axis_idx] > self.limits[ri][1])):
                if self.limits[ri][0] > self.limits[ri][1]:
                    raise move.move_error("Must home axis first")
                raise move.move_error()
    
    def check_move(self, move):
        """Check if move is valid"""
        limits = self.limits
        xpos, ypos = move.end_pos[:2]
        if (xpos < limits[0][0] or xpos > limits[0][1]
            or ypos < limits[1][0] or ypos > limits[1][1]):
            self._check_endstops(move)
        if not move.axes_d[2]:
            # No Z movement - validate other axes
            self._check_endstops(move)
            return
        # Move with Z - update velocity and accel for slower Z axis
        self._check_endstops(move)
        z_ratio = move.move_d / abs(move.axes_d[2])
        move.limit_speed(
            self.max_z_velocity * z_ratio, self.max_z_accel * z_ratio)
    
    def get_status(self, eventtime):
        """Get kinematics status"""
        axes_map = ['x', 'y', 'z', '', 'b', 'c']
        axes = [axes_map[i] for i, (l, h) in enumerate(self.limits) 
                if l <= h and axes_map[i]]
        return {
            'homed_axes': "".join(axes),
            'axis_minimum': self.axes_min,
            'axis_maximum': self.axes_max,
        }

def load_kinematics(toolhead, config):
    return DeltaBZCYXZKinematics(toolhead, config)