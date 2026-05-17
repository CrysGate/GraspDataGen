# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Gripper configurations for datagen.

This file contains predefined gripper configurations that can be used with the
--gripper_config argument. Each configuration can override any parameter from
gripper.py, grasp_guess.py, or grasp_sim.py.
"""

import sys
from graspgen_utils import print_blue

# Gripper configurations dictionary
GRIPPER_CONFIGS = {
    'robotiq_2f_85': {
        'gripper_file': 'bots/robotiq_2f_85.usd',
        'finger_colliders': ['right_inner_finger', 'left_inner_finger'],
        'base_frame': 'base_link',
        'bite': 0.0185,  # half of 37mm
    },
    'onrobot_rg6': {
        'gripper_file': 'bots/onrobot_rg6.usd',
        'finger_colliders': ['right_inner_finger', 'left_inner_finger'],
        'base_frame': 'base_frame',
        'bite': 0.025,
    },
    'franka_panda': {
        'gripper_file': 'bots/franka_panda.usd',
        'finger_colliders': ['panda_rightfinger', 'panda_leftfinger'],
        'base_frame': 'panda_hand',
    },
    'piper_v2_gripper': {
        'gripper_file': 'bots/piper_v2_gripper.usd',
        'finger_colliders': ['link7', 'link8'],
        'base_frame': 'link6',
        'bite': 0.02,
        'pinch_width_resolution': 8,
    },
    'piper_h_v1_gripper': {
        'gripper_file': 'bots/piper_h_v1_gripper.usd',
        'finger_colliders': ['Link7', 'Link8'],
        'base_frame': 'Link6',
        'bite': 0.02,
        'pinch_width_resolution': 8,
    },
    'piper_l_v1_gripper': {
        'gripper_file': 'bots/piper_l_v1_gripper.usd',
        'finger_colliders': ['Link7', 'Link8'],
        'base_frame': 'Link6',
        'bite': 0.02,
        'pinch_width_resolution': 8,
    },
    'piper_v1_gripper': {
        'gripper_file': 'bots/piper_v1_gripper.usd',
        'finger_colliders': ['link7', 'link8'],
        'base_frame': 'link6',
        'bite': 0.02,
        'pinch_width_resolution': 8,
    },
    'piper_x_v1_gripper': {
        'gripper_file': 'bots/piper_x_v1_gripper.usd',
        'finger_colliders': ['Link7', 'Link8'],
        'base_frame': 'Link6',
        'bite': 0.02,
        'pinch_width_resolution': 8,
    },
    'changingtek_ag2f120s': {
        'gripper_file': 'bots/changingtek_ag2f120s.usd',
        'finger_colliders': ['left_support_link', 'right_support_link'],
        'base_frame': 'gripper_base',
        'bite': 0.02,
        'pinch_width_resolution': 8,
    },
    'changingtek_ag2f90': {
        'gripper_file': 'bots/changingtek_ag2f90.usd',
        'finger_colliders': ['left_up_link', 'right_up_link'],
        'base_frame': 'flange',
        'bite': 0.02,
        'pinch_width_resolution': 8,
    },
    'galaxea_g1': {
        'gripper_file': 'bots/galaxea_g1.usd',
        'finger_colliders': ['finger1', 'finger2'],
        'base_frame': 'gripper_base',
        'bite': 0.015,
        'pinch_width_resolution': 8,
    },
    'inspire_eg2_4c2': {
        'gripper_file': 'bots/inspire_eg2_4c2.usd',
        'finger_colliders': ['left_pad', 'right_pad'],
        'base_frame': 'gripper_base',
        'bite': 0.012,
        'pinch_width_resolution': 8,
    },
    'omnipicker': {
        'gripper_file': 'bots/omnipicker.usd',
        'finger_colliders': ['narrow3_Link', 'wide3_Link'],
        'base_frame': 'gripper_base',
        'bite': 0.02,
        'pinch_width_resolution': 8,
    },
    'robotiq_85': {
        'gripper_file': 'bots/robotiq_85.usd',
        'finger_colliders': ['right_inner_finger', 'left_inner_finger'],
        'base_frame': 'gripper_base',
        'bite': 0.0185,
        'pinch_width_resolution': 8,
    },
    'robot_g1_120s_gripper': {
        'gripper_file': 'bots/robot_g1_120s_gripper.usd',
        'finger_colliders': ['gripper_r_inner_link5', 'gripper_r_outer_link5'],
        'base_frame': 'gripper_r_base_link',
        'bite': 0.018,
        'pinch_width_resolution': 8,
    },
    'robot_g1_omnipicker_gripper': {
        'gripper_file': 'bots/robot_g1_omnipicker_gripper.usd',
        'finger_colliders': ['gripper_r_inner_link4', 'gripper_r_outer_link4'],
        'base_frame': 'gripper_r_base_link',
        'bite': 0.016,
        'pinch_width_resolution': 8,
    },
    'robot_g2_90d_gripper': {
        'gripper_file': 'bots/robot_g2_90d_gripper.usd',
        'finger_colliders': ['gripper_r_left_support_link', 'gripper_r_right_support_link'],
        'base_frame': 'gripper_r_base_link',
        'bite': 0.018,
        'pinch_width_resolution': 8,
    },
    'robot_g2_omnipicker_gripper': {
        'gripper_file': 'bots/robot_g2_omnipicker_gripper.usd',
        'finger_colliders': ['gripper_r_inner_link4', 'gripper_r_outer_link4'],
        'base_frame': 'gripper_r_base_link',
        'bite': 0.016,
        'pinch_width_resolution': 8,
    },
    'robot_g2_place_workpiece_gripper': {
        'gripper_file': 'bots/robot_g2_place_workpiece_gripper.usd',
        'finger_colliders': ['gripper_r_inner_link1', 'gripper_r_outer_link1'],
        'base_frame': 'gripper_r_base_link',
        'bite': 0.012,
        'pinch_width_resolution': 8,
    },
}


def get_gripper_config(gripper_type):
    """
    Get gripper configuration based on gripper type string.

    Args:
        gripper_type (str): One of the predefined gripper types

    Returns:
        dict: Dictionary containing parameter overrides for the specified
        gripper type

    Raises:
        ValueError: If gripper_type is not found in configurations
    """
    if gripper_type not in GRIPPER_CONFIGS:
        available = list(GRIPPER_CONFIGS.keys())
        raise ValueError(f"Unknown gripper type: {gripper_type}. "
                        f"Must be one of: {available}")

    return GRIPPER_CONFIGS[gripper_type]

def apply_gripper_config_to_args(args, gripper_config):
    """
    Apply gripper configuration overrides to the args object.
    Only overrides values that weren't explicitly provided on the command line.

    Args:
        args: The parsed arguments object
        gripper_config: Dictionary of parameter overrides
    """
    # Apply each override to the args object
    for param_name, value in gripper_config.items():
        if hasattr(args, param_name):
            # Check if this argument was explicitly provided on command line
            arg_flag = f"--{param_name}"
            if arg_flag in sys.argv:
                # Find the index of the flag in sys.argv
                try:
                    flag_index = sys.argv.index(arg_flag)
                    # Check if there's a value after the flag (not another flag)
                    if flag_index + 1 < len(sys.argv) and not sys.argv[flag_index + 1].startswith('-'):
                        print_blue(f"  Skipping {param_name}: {value} (explicitly provided on command line)")
                        continue
                except ValueError:
                    pass  # Flag not found, continue with override

            setattr(args, param_name, value)
            print_blue(f"  Overriding {param_name}: {value}")
        else:
            # Let argparse handle unknown parameters - this will raise an error
            raise ValueError(f"Unknown parameter '{param_name}' in gripper config "
                           f"'{args.gripper_config}'")


def list_available_grippers():
    """
    List all available gripper configurations.

    Returns:
        list: List of available gripper type names
    """
    return list(GRIPPER_CONFIGS.keys())
