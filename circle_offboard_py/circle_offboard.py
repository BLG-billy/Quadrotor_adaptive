#!/usr/bin/env python3
"""Start the modular experiment using PX4's built-in position controller."""
from .controller_experiment import main as start_experiment


def main(args=None):
    start_experiment(args=args, control_mode='position')


if __name__ == '__main__':
    main()
