defs_kin_rotary_delta = """
    struct stepper_kinematics *rotary_delta_stepper_alloc(
        double shoulder_radius, double shoulder_height
        , double angle, double upper_arm, double lower_arm);
    struct stepper_kinematics *rotary_delta_bed_stepper_alloc(void);
"""