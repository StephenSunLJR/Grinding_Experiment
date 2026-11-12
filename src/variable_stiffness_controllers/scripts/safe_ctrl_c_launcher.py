#!/usr/bin/env python3
"""Launch the controller while turning the first Ctrl+C into a safe return."""

import argparse
import os
import signal
import subprocess
import sys
import threading
import time


def call_trigger(service_name, timeout=3.0):
    try:
        result = subprocess.run(
            ["rosservice", "call", service_name],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            timeout=timeout,
            check=False,
        )
        return result.returncode == 0, result.stdout
    except (OSError, subprocess.TimeoutExpired) as exception:
        return False, str(exception)


def stop_process_group(process):
    if process.poll() is not None:
        return
    for requested_signal, timeout in (
        (signal.SIGINT, 10.0),
        (signal.SIGTERM, 3.0),
        (signal.SIGKILL, 1.0),
    ):
        try:
            os.killpg(process.pid, requested_signal)
            process.wait(timeout=timeout)
            return
        except ProcessLookupError:
            return
        except subprocess.TimeoutExpired:
            continue


def main():
    parser = argparse.ArgumentParser(
        description="Intercept Ctrl+C, return Franka to its controller start pose, then stop roslaunch."
    )
    parser.add_argument("--return-timeout", type=float, default=20.0)
    parser.add_argument(
        "--controller-name", default="bidirectional_energy_flow_control"
    )
    arguments, roslaunch_arguments = parser.parse_known_args()
    if arguments.return_timeout <= 0.0:
        parser.error("--return-timeout must be positive")

    interrupt_requested = threading.Event()
    immediate_shutdown_requested = threading.Event()
    interrupt_count = [0]

    def handle_sigint(_signal_number, _frame):
        interrupt_count[0] += 1
        if interrupt_count[0] == 1:
            print("\nCtrl+C received: requesting safe return. Press Ctrl+C again for immediate shutdown.")
            interrupt_requested.set()
        else:
            print("\nSecond Ctrl+C received: stopping immediately.")
            immediate_shutdown_requested.set()

    signal.signal(signal.SIGINT, handle_sigint)
    command = [
        "roslaunch",
        "variable_stiffness_controllers",
        "bidirectional_energy_flow_control.launch",
    ] + roslaunch_arguments
    try:
        launch_process = subprocess.Popen(command, start_new_session=True)
    except OSError as exception:
        print("Failed to start roslaunch: {}".format(exception), file=sys.stderr)
        return 1

    try:
        while launch_process.poll() is None and not interrupt_requested.wait(0.1):
            pass
        if launch_process.poll() is not None:
            return launch_process.returncode

        namespace = "/" + arguments.controller_name.strip("/")
        request_service = namespace + "/request_safe_stop"
        status_service = namespace + "/safe_stop_status"
        request_ok, request_output = call_trigger(request_service)
        request_accepted = request_ok and (
            "success: True" in request_output or "success: true" in request_output
        )
        if not request_accepted:
            print(
                "Safe-return request failed; stopping control immediately:\n{}".format(
                    request_output.strip()
                ),
                file=sys.stderr,
            )
        else:
            deadline = time.monotonic() + arguments.return_timeout
            completed = False
            while (
                time.monotonic() < deadline
                and launch_process.poll() is None
                and not immediate_shutdown_requested.is_set()
            ):
                status_ok, status_output = call_trigger(status_service, timeout=1.0)
                if status_ok and (
                    "success: True" in status_output
                    or "success: true" in status_output
                ):
                    completed = True
                    print("Safe return completed; shutting down roslaunch.")
                    break
                time.sleep(0.2)
            if not completed and not immediate_shutdown_requested.is_set():
                print(
                    "Safe return timed out; stopping control so the robot brakes.",
                    file=sys.stderr,
                )
        stop_process_group(launch_process)
        return launch_process.returncode or 0
    finally:
        stop_process_group(launch_process)


if __name__ == "__main__":
    sys.exit(main())
