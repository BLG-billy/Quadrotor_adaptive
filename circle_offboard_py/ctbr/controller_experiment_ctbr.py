#!/usr/bin/env python3
"""Start the same circle experiment with PX4 body-rate control."""
from ..controller_experiment import main as start_experiment


def main(args=None):
    start_experiment(args=args, control_mode='ctbr')


if __name__ == '__main__':
    main()
