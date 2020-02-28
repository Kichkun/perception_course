#!/usr/bin/python

"""
Sudhanva Sreesha
ssreesha@umich.edu
21-Apr-2018

Gonzalo Ferrer
g.ferrer@skoltech.ru
26-Nov-2018

Script to generate data for SLAM.
The call to this function should be:
python generate_data.py -o ./slam-evaluation-input.npy --animate -n 200
"""

from warnings import warn

import numpy as np

from argparse import ArgumentParser
from tools.data import generate_data
from tools.data import save_data


def get_cli_args():
    parser = ArgumentParser('Perception in Robotics PS3, Generate data to run the state estimation simulation.')
    parser.add_argument('--initial-robot-pose',
                        nargs=3,
                        metavar=('X', 'Y', 'THETA'),
                        dest='initial_robot_pose',
                        help='The initial pose of the robot (format: x y theta).',
                        default=(180, 50, 0))
    parser.add_argument('-a',
                        '--alphas',
                        nargs=4,
                        action='store',
                        metavar=('A1', 'A2', 'A3', 'A4'),
                        help='Motion noise. (format: a1 a2 a3 a4).',
                        default=(0.05, 0.001, 0.05, 0.01))
    parser.add_argument('-b',
                        '--beta',
                        nargs=2,
                        type=float,
                        help='Observation noise (format: cm deg).',
                        default=(10., 10.))
    parser.add_argument('-t', '--dt', type=float, help='Time step (in seconds).', default=0.1)
    parser.add_argument('--num-landmarks-per-side',
                        type=int,
                        help='The number of landmarks to generate on one side of the field.',
                        default=4)
    parser.add_argument('-n',
                        '--num-steps',
                        type=int,
                        action='store',
                        help='The number of time step to generate the data for.',
                        default=200)
    parser.add_argument('--max-obs-per-time-step',
                        type=int,
                        help='The maximum number of observations to generate per time step.',
                        default=2)
    parser.add_argument('-o',
                        '--output-file-path',
                        type=str,
                        action='store',
                        help='The the full path to which the generate data will be saved.',
                        default=None)
    parser.add_argument('--animate',
                        action='store_true',
                        help='Show and animation of the generated data, in real-time.')
    parser.add_argument('--plot-pause-len',
                        type=float,
                        action='store',
                        help='Time (in seconds) to pause the plot animation for between frames.',
                        default=0.01)
    return parser.parse_args()


def main():
    args = get_cli_args()
    if not args.animate and not args.output_file_path:
        warn('Since both --animate and --output-file-path options were not specified, exiting data generation.')
        return

    data = generate_data(args.initial_robot_pose,
                         args.num_steps,
                         args.num_landmarks_per_side,
                         args.max_obs_per_time_step,
                         np.array(args.alphas),
                         args.beta,
                         args.dt,
                         args.animate,
                         args.plot_pause_len)

    if not args.output_file_path:
        warn('Generated data is not being saved.', RuntimeWarning)
        return

    save_data(data, args.output_file_path)


if __name__ == '__main__':
    main()
