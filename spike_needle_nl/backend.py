"""Where the tool calls land: a mock, and the real Nav2 bridge.

The eval runs against `MockBackend` -- the question it asks is whether the model
picks the right call, which does not depend on Gazebo being up. `Nav2Backend` is
the real thing and every one of its `act` methods moves a physical robot, which is
why the console gates them and why the eval can never reach it by accident.
"""

import math
import threading

import tb_tools


class MockBackend:
    name = "mock"
    live = False

    def __init__(self):
        self.pose = (0.15, -0.60, 0.0)
        self.goal = None
        self.max_speed = 0.22
        self.writes = []

    def robot_pose(self):
        x, y, yaw = self.pose
        return {"x": x, "y": y, "yaw_deg": round(math.degrees(yaw), 1),
                "nearest_place": "hallway", "frame": "map"}

    def navigation_status(self):
        if self.goal is None:
            return {"navigating": False, "goal": None, "state": "idle"}
        return {"navigating": True, "goal": self.goal, "state": "driving",
                "distance_remaining_m": 1.8}

    def list_places(self):
        return {"places": sorted(tb_tools.PLACES)}

    def nearest_obstacle(self):
        return {"distance_m": 0.47, "direction": "front-left", "bearing_deg": 34}

    def set_max_speed(self, speed):
        old, self.max_speed = self.max_speed, speed
        self.writes.append(("set_max_speed", speed))
        return {"ok": True, "old": old, "new": speed}

    def go_to_place(self, place):
        self.goal = place
        self.writes.append(("go_to_place", place))
        return {"ok": True, "goal": place, "pose": tb_tools.PLACES[place],
                "accepted": True}

    def stop(self):
        cancelled, self.goal = self.goal, None
        self.writes.append(("stop",))
        return {"ok": True, "cancelled_goal": cancelled}

    def turn(self, direction, degrees):
        self.writes.append(("turn", direction, degrees))
        return {"ok": True, "direction": direction, "degrees": degrees}

    def drive(self, direction, distance):
        self.writes.append(("drive", direction, distance))
        return {"ok": True, "direction": direction, "distance_m": distance}


class Nav2Backend:
    """Real bridge: navigate_to_pose, /cmd_vel, /amcl_pose, /scan, Nav2 params.

    NOT exercised against a running stack in this spike. `turn` and `drive` are
    open-loop timed /cmd_vel bursts, which is fine for a nudge and wrong for
    anything else -- Nav2's BackUp and Spin behaviours are the right answer if
    these ever become more than a demo.
    """

    name = "nav2"
    live = True

    def __init__(self, timeout=5.0):
        import rclpy
        from rclpy.action import ActionClient
        from rclpy.callback_groups import ReentrantCallbackGroup
        from rclpy.executors import MultiThreadedExecutor
        from rclpy.node import Node

        from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
        from nav2_msgs.action import NavigateToPose
        from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                               ReliabilityPolicy, qos_profile_sensor_data)
        from sensor_msgs.msg import LaserScan

        if not rclpy.ok():
            rclpy.init()
        self.rclpy = rclpy
        self.timeout = timeout
        self.node = Node("needle_tb_bridge")
        self.cb = ReentrantCallbackGroup()
        self._lock = threading.Lock()
        self._pose = None
        self._scan = None
        self._goal_handle = None
        self._goal_name = None

        self._nav = ActionClient(self.node, NavigateToPose, "navigate_to_pose",
                                 callback_group=self.cb)
        self._cmd = self.node.create_publisher(Twist, "cmd_vel", 10)
        # Both of these QoS profiles are load-bearing, and both were found by
        # running against a live stack -- the mock backend cannot surface either.
        #
        # amcl_pose is published TRANSIENT_LOCAL and only when the pose estimate
        # updates. A VOLATILE subscriber that joins late therefore gets nothing at
        # all while the robot sits still, which reads exactly like "localisation
        # is down". Matching the durability delivers the last sample on subscribe.
        amcl_qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                              reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.node.create_subscription(
            PoseWithCovarianceStamped, "amcl_pose", self._on_pose, amcl_qos,
            callback_group=self.cb)
        # The bridge publishes scan BEST_EFFORT (ignition_sim.launch.py sets that
        # override explicitly). A default RELIABLE subscriber is INCOMPATIBLE and
        # silently receives nothing; rclpy only warns.
        self.node.create_subscription(
            LaserScan, "scan", self._on_scan, qos_profile_sensor_data,
            callback_group=self.cb)

        self._exec = MultiThreadedExecutor()
        self._exec.add_node(self.node)
        threading.Thread(target=self._exec.spin, daemon=True).start()

    def _on_pose(self, msg):
        with self._lock:
            self._pose = msg

    def _on_scan(self, msg):
        with self._lock:
            self._scan = msg

    @staticmethod
    def _yaw(q):
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                          1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def robot_pose(self):
        with self._lock:
            msg = self._pose
        if msg is None:
            return {"error": "no pose on /amcl_pose; is localisation up?"}
        p = msg.pose.pose
        yaw = self._yaw(p.orientation)
        nearest = min(tb_tools.PLACES,
                      key=lambda k: (tb_tools.PLACES[k][0] - p.position.x) ** 2
                      + (tb_tools.PLACES[k][1] - p.position.y) ** 2)
        return {"x": round(p.position.x, 2), "y": round(p.position.y, 2),
                "yaw_deg": round(math.degrees(yaw), 1),
                "nearest_place": nearest, "frame": "map"}

    # action_msgs/GoalStatus. Anything from SUCCEEDED on is terminal: the goal is
    # finished and the robot is NOT driving. Reporting "navigating" for those was
    # a real bug -- a cancelled or aborted goal read as still in progress, which
    # is exactly backwards for the one question an operator asks after a failure.
    _STATUS = {1: ("accepted", True), 2: ("driving", True), 3: ("cancelling", True),
               4: ("arrived", False), 5: ("cancelled", False),
               6: ("failed", False)}

    def navigation_status(self):
        handle = self._goal_handle
        if handle is None:
            return {"navigating": False, "goal": None, "state": "idle"}
        state, moving = self._STATUS.get(handle.status, ("unknown", False))
        if not moving:
            # Terminal: forget it so the next question reads "idle" rather than
            # re-reporting a goal that ended minutes ago.
            self._goal_handle, name = None, self._goal_name
            self._goal_name = None
            return {"navigating": False, "goal": name, "state": state}
        return {"navigating": True, "goal": self._goal_name, "state": state}

    def list_places(self):
        return {"places": sorted(tb_tools.PLACES)}

    def nearest_obstacle(self):
        with self._lock:
            scan = self._scan
        if scan is None:
            return {"error": "no scan on /scan"}
        best, best_i = None, None
        for i, r in enumerate(scan.ranges):
            if scan.range_min < r < scan.range_max and (best is None or r < best):
                best, best_i = r, i
        if best is None:
            return {"error": "no valid returns in the scan"}
        bearing = math.degrees(scan.angle_min + best_i * scan.angle_increment)
        return {"distance_m": round(best, 2), "bearing_deg": round(bearing, 1)}

    def set_max_speed(self, speed):
        from rcl_interfaces.srv import SetParameters
        from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
        client = self.node.create_client(
            SetParameters, "/controller_server/set_parameters")
        if not client.wait_for_service(timeout_sec=self.timeout):
            return {"error": "/controller_server is not up"}
        request = SetParameters.Request()
        for name in ("FollowPath.max_vel_x", "FollowPath.max_speed_xy"):
            value = ParameterValue()
            value.type = ParameterType.PARAMETER_DOUBLE
            value.double_value = float(speed)
            request.parameters.append(Parameter(name=name, value=value))
        future = client.call_async(request)
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        done.wait(self.timeout)
        if not future.done():
            return {"error": "/controller_server did not answer"}
        results = future.result().results
        return {"ok": all(r.successful for r in results), "new": speed,
                "reasons": [r.reason for r in results if not r.successful]}

    def go_to_place(self, place):
        from geometry_msgs.msg import PoseStamped
        from nav2_msgs.action import NavigateToPose
        if place not in tb_tools.PLACES:
            return {"error": f"unknown place {place}"}
        if not self._nav.wait_for_server(timeout_sec=self.timeout):
            return {"error": "navigate_to_pose action server is not up"}
        x, y, yaw = tb_tools.PLACES[place]
        goal = NavigateToPose.Goal()
        target = PoseStamped()
        target.header.frame_id = "map"
        target.header.stamp = self.node.get_clock().now().to_msg()
        target.pose.position.x, target.pose.position.y = x, y
        target.pose.orientation.z = math.sin(yaw / 2.0)
        target.pose.orientation.w = math.cos(yaw / 2.0)
        goal.pose = target

        future = self._nav.send_goal_async(goal)
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        done.wait(self.timeout)
        if not future.done():
            return {"error": "goal was never acknowledged"}
        handle = future.result()
        if not handle.accepted:
            return {"ok": False, "goal": place, "accepted": False}
        self._goal_handle, self._goal_name = handle, place
        return {"ok": True, "goal": place, "pose": (x, y, yaw), "accepted": True}

    def stop(self):
        from geometry_msgs.msg import Twist
        cancelled = self._goal_name
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
            self._goal_handle, self._goal_name = None, None
        # Zero the wheels too: cancelling a Nav2 goal does not by itself
        # guarantee the last published velocity stops being acted on.
        for _ in range(5):
            self._cmd.publish(Twist())
        return {"ok": True, "cancelled_goal": cancelled}

    def _burst(self, twist, seconds):
        import time
        from geometry_msgs.msg import Twist
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            self._cmd.publish(twist)
            time.sleep(0.05)
        self._cmd.publish(Twist())

    def turn(self, direction, degrees):
        from geometry_msgs.msg import Twist
        rate = 0.5
        twist = Twist()
        twist.angular.z = rate if direction == "left" else -rate
        self._burst(twist, math.radians(degrees) / rate)
        return {"ok": True, "direction": direction, "degrees": degrees,
                "note": "open-loop timed burst"}

    def drive(self, direction, distance):
        from geometry_msgs.msg import Twist
        speed = 0.15
        twist = Twist()
        twist.linear.x = speed if direction == "forward" else -speed
        self._burst(twist, distance / speed)
        return {"ok": True, "direction": direction, "distance_m": distance,
                "note": "open-loop timed burst"}

    def close(self):
        self._exec.shutdown()
        self.node.destroy_node()
