// Delta BZCYXZ kinematics stepper pulse time generation
//
// Copyright (C) 2025  Contributors
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <stdlib.h> // malloc
#include <string.h> // memset
#include <math.h>   // sin, cos, atan2, hypot
#include "compiler.h" // __visible
#include "itersolve.h" // struct stepper_kinematics
#include "pyhelper.h" // errorf
#include "trapq.h" // move_get_coord

struct deltabzcyxz_stepper {
    double offset_hypot;
    double offset_angle;
    char axis;
};

static double
deltabzcyxz_stepper_x_calc_position(struct stepper_kinematics *sk, struct move *m
                                     , double move_time)
{
    struct coord c = move_get_coord(m, move_time);
    struct deltabzcyxz_stepper *ds = sk->data;
    // X position with B-axis offset correction
    double b_rad = c.b * M_PI / 180.0 + ds->offset_angle;
    return c.x + ds->offset_hypot * cos(b_rad);
}

static double
deltabzcyxz_stepper_y_calc_position(struct stepper_kinematics *sk, struct move *m
                                     , double move_time)
{
    struct coord c = move_get_coord(m, move_time);
    // Y position (differential to C axis)
    return c.y;
}

static double
deltabzcyxz_stepper_z_calc_position(struct stepper_kinematics *sk, struct move *m
                                     , double move_time)
{
    struct coord c = move_get_coord(m, move_time);
    struct deltabzcyxz_stepper *ds = sk->data;
    // Z position with B-axis offset correction
    double b_rad = c.b * M_PI / 180.0 + ds->offset_angle;
    return c.z + ds->offset_hypot * sin(b_rad);
}

static double
deltabzcyxz_stepper_b_calc_position(struct stepper_kinematics *sk, struct move *m
                                     , double move_time)
{
    // B axis (pitch rotation around Y)
    return move_get_coord(m, move_time).b;
}

static double
deltabzcyxz_stepper_c_calc_position(struct stepper_kinematics *sk, struct move *m
                                     , double move_time)
{
    // C axis (yaw rotation around Z)
    return move_get_coord(m, move_time).c;
}

struct stepper_kinematics * __visible
deltabzcyxz_stepper_alloc(char axis, double offset_hypot, double offset_angle)
{
    struct stepper_kinematics *sk = malloc(sizeof(*sk));
    memset(sk, 0, sizeof(*sk));
    
    struct deltabzcyxz_stepper *ds = malloc(sizeof(*ds));
    memset(ds, 0, sizeof(*ds));
    ds->axis = axis;
    ds->offset_hypot = offset_hypot;
    ds->offset_angle = offset_angle;
    
    sk->data = ds;
    
    if (axis == 'x') {
        sk->calc_position_cb = deltabzcyxz_stepper_x_calc_position;
        sk->active_flags = AF_X;
    } else if (axis == 'y') {
        sk->calc_position_cb = deltabzcyxz_stepper_y_calc_position;
        sk->active_flags = AF_Y;
    } else if (axis == 'z') {
        sk->calc_position_cb = deltabzcyxz_stepper_z_calc_position;
        sk->active_flags = AF_Z;
    } else if (axis == 'b') {
        sk->calc_position_cb = deltabzcyxz_stepper_b_calc_position;
        sk->active_flags = AF_B;
    } else if (axis == 'c') {
        sk->calc_position_cb = deltabzcyxz_stepper_c_calc_position;
        sk->active_flags = AF_C;
    }
    return sk;
}

void __visible
deltabzcyxz_stepper_free(struct stepper_kinematics *sk)
{
    if (sk && sk->data)
        free(sk->data);
    if (sk)
        free(sk);
}
