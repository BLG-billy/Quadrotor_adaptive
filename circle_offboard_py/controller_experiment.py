#!/usr/bin/env python3
"""Start independent control, trajectory, flight management and recorder nodes."""
import signal
import subprocess
import sys
import time


def main(args=None, control_mode='wrench'):
    ros_args = sys.argv[1:] if args is None else args
    ros_args = list(ros_args)
    if control_mode not in ('wrench', 'position', 'ctbr'):
        raise ValueError(f'Unknown control mode: {control_mode}')
    if control_mode == 'ctbr':
        ros_args = ['--ros-args', '-p', 'experiment_name:=ctbr', *ros_args]
    ros_args += ['--ros-args', '-p', f'control_mode:={control_mode}']
    controller = 'controller' if control_mode == 'wrench' else control_mode

    def stop_on_signal(signum, frame):
        raise KeyboardInterrupt

    previous_handler = signal.signal(signal.SIGTERM, stop_on_signal)
    children = []
    try:
        for role in ('recorder', 'trajectory', controller, 'flight'):
            children.append(subprocess.Popen(
                [sys.executable, '-m', 'circle_offboard_py.node_runner', role, *ros_args],
                start_new_session=True))
        while True:
            for child in children:
                if child.poll() is not None:
                    raise RuntimeError(f'Experiment node exited with code {child.returncode}')
            time.sleep(0.2)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop command producers first; let the recorder flush remaining samples.
        for child in reversed(children):
            if child.poll() is None:
                child.send_signal(signal.SIGINT)
                try:
                    child.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    child.terminate()
                    try:
                        child.wait(timeout=3)
                    except subprocess.TimeoutExpired:
                        child.kill()
                        child.wait()

        signal.signal(signal.SIGTERM, previous_handler)


if __name__ == '__main__':
    main()
