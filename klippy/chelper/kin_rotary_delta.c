// Rotary delta kinematics stepper pulse time generation
//
// Copyright (C) 2019  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <math.h> // atan2, cos, sin, sqrt
#include <stdlib.h> // malloc
#include <string.h> // memset
#include "compiler.h" // __visible
#include "itersolve.h" // struct stepper_kinematics
#include "trapq.h" // move_get_coord

// Calculate upper arm angle from the elbow joint to the end effector joint.
static double
rotary_two_arm_calc(double dx, double dy, double upper_arm2,
                    double lower_arm2)
{
    double distance2 = dx*dx + dy*dy;
    double distance = sqrt(distance2);
    double along = (upper_arm2 + distance2 - lower_arm2) / (2. * distance);
    double height = sqrt(upper_arm2 - along*along);
    double elbow_x = along * dx / distance + height * dy / distance;
    double elbow_y = along * dy / distance - height * dx / distance;
    return atan2(elbow_y, elbow_x);
}

struct rotary_stepper {
    struct stepper_kinematics sk;
    double cos, sin, shoulder_radius, shoulder_height;
    double upper_arm2, lower_arm2;
};

static double
rotary_stepper_calc_position(struct stepper_kinematics *sk, struct move *m
                             , double move_time)
{
    struct rotary_stepper *rs = container_of(sk, struct rotary_stepper, sk);
    struct coord c = move_get_coord(m, move_time);
    // Transform bed-local XY coordinates into the fixed printer frame.
    double bed_angle = c.c * (M_PI / 180.);
    double bed_cos = cos(bed_angle), bed_sin = sin(bed_angle);
    double x = c.x * bed_cos - c.y * bed_sin;
    double y = c.x * bed_sin + c.y * bed_cos;
    // Rotate and shift axes to an origin at the shoulder joint with upper
    // arm constrained to xy plane and x aligned to shoulder platform.
    double sjz = y * rs->cos - x * rs->sin;
    double sjx = x * rs->cos + y * rs->sin - rs->shoulder_radius;
    double sjy = c.z - rs->shoulder_height;
    return rotary_two_arm_calc(sjx, sjy, rs->upper_arm2,
                               rs->lower_arm2 - sjz*sjz);
}

struct stepper_kinematics * __visible
rotary_delta_stepper_alloc(double shoulder_radius, double shoulder_height
                           , double angle, double upper_arm, double lower_arm)
{
    struct rotary_stepper *rs = malloc(sizeof(*rs));
    memset(rs, 0, sizeof(*rs));
    rs->cos = cos(angle);
    rs->sin = sin(angle);
    rs->shoulder_radius = shoulder_radius;
    rs->shoulder_height = shoulder_height;
    rs->upper_arm2 = upper_arm * upper_arm;
    rs->lower_arm2 = lower_arm * lower_arm;
    rs->sk.calc_position_cb = rotary_stepper_calc_position;
    rs->sk.active_flags = AF_X | AF_Y | AF_Z | AF_C;
    return &rs->sk;
}

// The bed axis is configured in radians internally, while G-Code C values
// are expressed in degrees.
static double
rotary_delta_bed_calc_position(struct stepper_kinematics *sk, struct move *m
                               , double move_time)
{
    struct coord c = move_get_coord(m, move_time);
    return c.c * (M_PI / 180.);
}

struct stepper_kinematics * __visible
rotary_delta_bed_stepper_alloc(void)
{
    struct stepper_kinematics *sk = malloc(sizeof(*sk));
    memset(sk, 0, sizeof(*sk));
    sk->calc_position_cb = rotary_delta_bed_calc_position;
    sk->active_flags = AF_C;
    return sk;
}
