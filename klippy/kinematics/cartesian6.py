# Code for handling the kinematics of 6-axis cartesian robots
#
# Copyright (C) 2016-2025  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging
import stepper
from . import idex_modes

class Cart6Kinematics:
    def __init__(self, toolhead, config):
        self.printer = config.get_printer()
        self.rails = []
        for n in 'xyzabc':
            if config.has_section('stepper_' + n):
                self.rails.append(stepper.LookupMultiRail(config.getsection('stepper_' + n)))
            else:
                if n in 'xyz':
                    self.rails.append(stepper.LookupMultiRail(config.getsection('stepper_' + n)))
                else:
                    self.rails.append(None)

        for i, (rail, axis) in enumerate(zip(self.rails, 'xyzabc')):
            if rail is not None:
                rail.setup_itersolve('cartesian_6axis_stepper_alloc', axis.encode())
        ranges = [(0., 0.) if r is None else r.get_range() for r in self.rails]
        # We extend the toolhead.Coord to use min/max of all 6 axes
        self.axes_min = toolhead.Coord([r[0] for r in ranges[:3]])
        self.axes_max = toolhead.Coord([r[1] for r in ranges[:3]])
        self.axes_min_6 = [r[0] for r in ranges]
        self.axes_max_6 = [r[1] for r in ranges]
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
        return [s for rail in self.rails if rail is not None for s in rail.get_steppers()]

    def calc_position(self, stepper_positions):
        return [0.0 if rail is None else stepper_positions[rail.get_name()] for rail in self.rails]

    def update_limits(self, i, range):
        l, h = self.limits[i]
        if l <= h:
            self.limits[i] = range

    def set_position(self, newpos, homing_axes):
        for i, rail in enumerate(self.rails):
            if rail is not None:
                rail.set_position(newpos)
        for axis_name in homing_axes:
            axis = "xyzabc".index(axis_name)
            rail = self.rails[axis]
            if rail is not None:
                self.limits[axis] = rail.get_range()

    def clear_homing_state(self, clear_axes):
        for axis, axis_name in enumerate("xyzabc"):
            if axis_name in clear_axes:
                self.limits[axis] = (1.0, -1.0)

    def home_axis(self, homing_state, axis, rail):
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
        for axis in homing_state.get_axes():
            if self.rails[axis] is not None:
                self.home_axis(homing_state, axis, self.rails[axis])

    def _check_endstops(self, move):
        end_pos = move.end_pos
        for i in range(6):
            if (move.axes_d[i]
                and (end_pos[i] < self.limits[i][0]
                     or end_pos[i] > self.limits[i][1])):
                if self.limits[i][0] > self.limits[i][1]:
                    raise move.move_error("Must home axis first")
                raise move.move_error()

    def check_move(self, move):
        limits = self.limits
        end_pos = move.end_pos
        # Validate all 6 axes are within boundaries
        for i in range(6):
            if (end_pos[i] < limits[i][0] or end_pos[i] > limits[i][1]):
                self._check_endstops(move)

        if not move.axes_d[2]:
            return

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
    # Check if 'stepper_a', 'stepper_b', or 'stepper_c' are in config
    if (not config.has_section('stepper_a') and
        not config.has_section('stepper_b') and
        not config.has_section('stepper_c')):
        from . import cartesian
        return cartesian.load_kinematics(toolhead, config)
    return Cart6Kinematics(toolhead, config)
