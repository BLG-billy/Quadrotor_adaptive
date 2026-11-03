"""Run the wrench experiment with L1 augmentation enabled by default."""
import sys

from .controller_experiment import main as start_experiment


def main(args=None):
    args = list(sys.argv[1:] if args is None else args)
    start_experiment(args=[
        '--ros-args', '-p', 'enable_l1:=true', '-p', 'experiment_name:=geometric_l1',
        *args], control_mode='wrench')


if __name__ == '__main__':
    main()
