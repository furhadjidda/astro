#!/usr/bin/env python3
"""
micro_ros_agent watchdog for ROS 2 Humble.

Monitors a heartbeat topic and restarts the micro_ros_agent process
if no message is received within the timeout window.

Usage:
    python3 microros_watchdog.py [OPTIONS]

    --topic         Topic to monitor       (default: /bno055_imu_raw)
    --msg-type      Message type           (default: sensor_msgs/msg/Imu)
    --timeout       Silence timeout (sec)  (default: 5.0)
    --port          Agent UDP port         (default: 8888)
    --transport     Agent transport        (default: udp4)
    --cooldown      Seconds to wait after killing before restarting (default: 3.0)
"""

import argparse
import errno
import socket
import signal
import subprocess
import threading
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from sensor_msgs.msg import Imu, NavSatFix

# ── topic-type registry ────────────────────────────────────────────────────────
MSG_TYPE_MAP = {
    "sensor_msgs/msg/Imu": Imu,
    "sensor_msgs/msg/NavSatFix": NavSatFix,
}

# micro_ros default QoS is best-effort; match it so we actually receive msgs
BEST_EFFORT_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)


class HeartbeatWatchdog(Node):
    """
    ROS 2 node that watches a topic and triggers a callback
    when the heartbeat goes silent for longer than `timeout` seconds.
    """

    def __init__(self, topic: str, msg_class, timeout: float, on_timeout):
        super().__init__("microros_watchdog")

        self._timeout = timeout
        self._on_timeout = on_timeout
        self._last_msg_time = time.monotonic()
        self._timed_out = False

        self.get_logger().info(f"Watching '{topic}' (timeout={timeout}s)")

        self.create_subscription(
            msg_class,
            topic,
            self._msg_callback,
            BEST_EFFORT_QOS,
        )

        # Check the heartbeat every 1 second
        self.create_timer(1.0, self._check_heartbeat)

    # ── callbacks ─────────────────────────────────────────────────────────────

    def _msg_callback(self, _msg):
        now = time.monotonic()
        if self._timed_out:
            self.get_logger().info("Heartbeat restored.")
            self._timed_out = False
        self._last_msg_time = now

    def _check_heartbeat(self):
        elapsed = time.monotonic() - self._last_msg_time
        self.get_logger().debug(f"Last message {elapsed:.1f}s ago")

        if elapsed >= self._timeout and not self._timed_out:
            self._timed_out = True
            self.get_logger().warning(
                f"No heartbeat for {elapsed:.1f}s — triggering restart."
            )
            # Fire the callback in a separate thread so we don't block the executor
            threading.Thread(target=self._on_timeout, daemon=True).start()


# ── agent process management ───────────────────────────────────────────────────


class AgentManager:
    """Manages the lifecycle of the micro_ros_agent process."""

    def __init__(self, transport: str, port: int, cooldown: float):
        self._transport = transport
        self._port = port
        self._cooldown = cooldown
        self._proc: subprocess.Popen | None = None
        self._lock = threading.Lock()

    def _udp_port_in_use(self) -> bool:
        """Return True if another process is already bound to this UDP port."""
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            sock.bind(("0.0.0.0", self._port))
            return False
        except OSError as exc:
            if exc.errno == errno.EADDRINUSE:
                return True
            raise
        finally:
            sock.close()

    def start(self):
        with self._lock:
            if self._proc and self._proc.poll() is None:
                print(
                    f"[watchdog] Agent already running (pid={self._proc.pid})",
                    flush=True,
                )
                return
            if self._udp_port_in_use():
                print(
                    f"[watchdog] Cannot start agent: UDP port {self._port} is already in use.",
                    flush=True,
                )
                print(
                    "[watchdog] Another micro_ros_agent (or service) is likely already running.",
                    flush=True,
                )
                return
            cmd = [
                "ros2",
                "run",
                "micro_ros_agent",
                "micro_ros_agent",
                self._transport,
                "--port",
                str(self._port),
            ]
            print(f"[watchdog] Starting agent: {' '.join(cmd)}", flush=True)
            # Inherit parent stdout/stderr so agent logs are visible in this terminal.
            self._proc = subprocess.Popen(cmd)
            # Give the process a brief moment to fail fast (e.g. bind error).
            time.sleep(0.25)
            ret = self._proc.poll()
            if ret is not None:
                print(
                    f"[watchdog] Agent exited immediately (code={ret}). Check logs above.",
                    flush=True,
                )
                self._proc = None
                return
            print(f"[watchdog] Agent started  (pid={self._proc.pid})", flush=True)

    def stop(self):
        with self._lock:
            if self._proc is None:
                return
            pid = self._proc.pid
            if self._proc.poll() is None:
                print(f"[watchdog] Killing agent (pid={pid})", flush=True)
                self._proc.terminate()
                try:
                    self._proc.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    print(
                        f"[watchdog] SIGTERM ignored — sending SIGKILL to {pid}",
                        flush=True,
                    )
                    self._proc.kill()
                    self._proc.wait()
            self._proc = None
            print(f"[watchdog] Agent stopped  (pid={pid})", flush=True)

    def restart(self):
        print("[watchdog] Restarting micro_ros_agent ...", flush=True)
        self.stop()
        print(f"[watchdog] Waiting {self._cooldown}s before restart ...", flush=True)
        time.sleep(self._cooldown)
        self.start()

    def is_running(self) -> bool:
        with self._lock:
            return self._proc is not None and self._proc.poll() is None


# ── entry point ───────────────────────────────────────────────────────────────


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="micro_ros_agent watchdog — restarts the agent on heartbeat loss."
    )
    parser.add_argument(
        "--topic",
        default="/bno055_imu_raw",
        help="Topic to monitor (default: /bno055_imu_raw)",
    )
    parser.add_argument(
        "--msg-type",
        default="sensor_msgs/msg/Imu",
        choices=list(MSG_TYPE_MAP.keys()),
        help="Message type of the heartbeat topic",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=5.0,
        help="Seconds of silence before restart (default: 5)",
    )
    parser.add_argument(
        "--port",
        type=int,
        default=8888,
        help="UDP port for the micro_ros_agent (default: 8888)",
    )
    parser.add_argument(
        "--transport",
        default="udp4",
        help="Transport type for the agent (default: udp4)",
    )
    parser.add_argument(
        "--cooldown",
        type=float,
        default=3.0,
        help="Seconds to wait between kill and restart (default: 3)",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    msg_class = MSG_TYPE_MAP[args.msg_type]

    agent = AgentManager(
        transport=args.transport,
        port=args.port,
        cooldown=args.cooldown,
    )

    # Start the agent immediately on launch
    agent.start()

    # Give the agent a moment to bind its socket before we start listening
    time.sleep(2.0)

    rclpy.init()

    # Callback fired by the watchdog node when the heartbeat goes silent
    def on_timeout():
        agent.restart()

    node = HeartbeatWatchdog(
        topic=args.topic,
        msg_class=msg_class,
        timeout=args.timeout,
        on_timeout=on_timeout,
    )

    shutdown_requested = threading.Event()

    # Keep signal handlers minimal: request shutdown and let the main cleanup path run once.
    def request_shutdown(signum, _frame):
        if shutdown_requested.is_set():
            return
        shutdown_requested.set()
        signal_name = signal.Signals(signum).name
        print(f"\n[watchdog] {signal_name} received, shutting down ...", flush=True)
        if rclpy.ok():
            rclpy.shutdown()

    signal.signal(signal.SIGINT, request_shutdown)
    signal.signal(signal.SIGTERM, request_shutdown)

    print("[watchdog] Spinning - press Ctrl-C to stop.", flush=True)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n[watchdog] Ctrl-C received, shutting down ...", flush=True)
    except ExternalShutdownException:
        # Expected when shutdown() is called while spin() is waiting.
        pass
    except Exception as exc:
        print(f"[watchdog] Unexpected error: {exc}", flush=True)
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        try:
            node.destroy_node()
        except Exception:
            pass
        agent.stop()


if __name__ == "__main__":
    main()
