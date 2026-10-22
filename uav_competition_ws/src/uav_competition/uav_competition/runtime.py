"""Small subprocess helpers for Launch readiness and offline preparation.

The top-level Launch owns Gazebo, PX4 and Agent directly. Helpers never arm
the aircraft; start remains an explicit ROS service request.
"""
import argparse
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import yaml


def wait_listener(port, timeout):
    # Do not connect to MAVLink TCP: its accept() belongs exclusively to PX4.
    deadline = time.monotonic()+timeout
    while time.monotonic() < deadline:
        result = subprocess.run(['ss', '-H', '-ltn'], capture_output=True, text=True, check=True)
        if any(line.split()[3].rsplit(':', 1)[-1] == str(port) for line in result.stdout.splitlines() if len(line.split()) > 3):
            return
        time.sleep(.2)
    raise RuntimeError(f'Timed out waiting for simulator MAVLink listener TCP {port}')


def prepare(args):
    root, px4 = Path(args.project), Path(args.px4)
    if args.build:
        env = dict(os.environ, DONT_RUN='1', BUILD_JOBS=str(args.jobs))
        subprocess.run(['make', f'-j{args.jobs}', 'px4_sitl', 'gazebo-classic'], cwd=px4, env=env, check=True)
    binary = px4/'build/px4_sitl_default/bin/px4'
    if not binary.is_file():
        raise RuntimeError(f'PX4 binary missing: {binary}; launch with build:=true')
    if args.profile == 'vm':
        bundled = root/'uav_competition_ws/src/uav_competition/worlds/competition_course.world'
        if Path(args.world).resolve() != bundled.resolve():
            # Installed resources may be a copy of the bundled world.
            if Path(args.world).read_bytes() != bundled.read_bytes():
                raise RuntimeError('VM model transformation is only for the bundled training world; use profile:=standard for another scene')
        plugin = []
        if not args.headless:
            source, build = root/'tools/vm_gui', root/'tools/vm_gui/build'
            subprocess.run(['cmake', '-S', str(source), '-B', str(build), '-DCMAKE_BUILD_TYPE=Release'], check=True)
            subprocess.run(['cmake', '--build', str(build), '--parallel', str(args.jobs)], check=True)
            plugin = ['--gui-plugin', str(build/'libbao_vm_gui.so')]
        subprocess.run([sys.executable, str(root/'scripts/prepare_vm_scene.py'),
                        '--output', args.output, *plugin], check=True)


def px4_process(args):
    wait_listener(args.port, args.timeout)
    px4 = Path(args.px4)
    run = px4/'build/px4_sitl_default/rootfs'
    run.mkdir(parents=True, exist_ok=True)
    child = subprocess.Popen([str(px4/'build/px4_sitl_default/bin/px4'), '-d', '../etc'], cwd=run)
    def stop(signum, _frame):
        if child.poll() is None:
            child.send_signal(signum)
    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    try:
        return child.wait()
    finally:
        if child.poll() is None:
            child.terminate()
            try:
                child.wait(timeout=3)
            except subprocess.TimeoutExpired:
                child.kill()
                child.wait()


def wait_ready(args):
    import rclpy
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    interfaces = (yaml.safe_load(Path(args.interfaces).read_text()) or {}) if args.interfaces else {}
    remappings = interfaces.get('remappings', {})
    rclpy.init()
    node = rclpy.create_node('competition_readiness')
    state = {}
    subscriptions = [node.create_subscription(String, remappings.get(topic, topic),
                     lambda m,k=key: state.update({k:(time.monotonic(),m.data)}), 10)
                     for key, topic in [('flight','/uav/flight/state'),('preflight','/uav/preflight/report')]]
    deadline = time.monotonic()+args.timeout
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.2)
            if all(key in state and time.monotonic()-state[key][0] < 1.0 and state[key][1] == 'READY'
                   for key in ('flight', 'preflight')):
                service = remappings.get('/uav/mission/start', '/uav/mission/start')
                print(f'COMPETITION_READY: self-check passed; request {service} to fly', flush=True)
                if args.action == 'ready':
                    return 0
                client = node.create_client(Trigger, service)
                if not client.wait_for_service(timeout_sec=5.0):
                    raise RuntimeError(f'Mission service unavailable: {service}')
                future = client.call_async(Trigger.Request())
                rclpy.spin_until_future_complete(node, future, timeout_sec=8.0)
                if not future.done() or future.result() is None:
                    raise RuntimeError('Mission start response timed out; inspect mission state before retrying')
                response = future.result()
                print(f'success={response.success}, message={response.message}', flush=True)
                return 0 if response.success else 1
            if state.get('flight', (0, ''))[1] == 'FAILED':
                raise RuntimeError(f'Flight initialization failed: {state}')
        raise RuntimeError(f'Readiness timed out; no flight started; latest={state}')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='action', required=True)
    prep = sub.add_parser('prepare')
    prep.add_argument('--project', required=True)
    prep.add_argument('--px4', required=True)
    prep.add_argument('--world', required=True)
    prep.add_argument('--output', required=True)
    prep.add_argument('--profile', choices=['standard','vm'], default='standard')
    prep.add_argument('--build', action='store_true')
    prep.add_argument('--headless', action='store_true')
    prep.add_argument('--jobs', type=int, default=1)
    run = sub.add_parser('px4')
    run.add_argument('--px4', required=True)
    run.add_argument('--port', type=int, default=4560)
    run.add_argument('--timeout', type=float, default=90)
    for action in ('ready', 'start'):
        ready = sub.add_parser(action)
        ready.add_argument('--timeout', type=float, default=120)
        ready.add_argument('--interfaces', default='')
    args = parser.parse_args()
    try:
        if args.action == 'prepare':
            if args.jobs < 1:
                raise ValueError('jobs must be positive')
            prepare(args)
            return 0
        if args.action == 'px4':
            return px4_process(args)
        return wait_ready(args)
    except (RuntimeError, ValueError, OSError, subprocess.CalledProcessError) as exc:
        print(str(exc), file=sys.stderr, flush=True)
        return 1


if __name__ == '__main__':
    sys.exit(main())
