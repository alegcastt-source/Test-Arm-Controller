#!/usr/bin/env python3
"""Fetch velocity control organized around the six steps in your notes.

Replace scripts/twist_controller.py with this file. It uses the existing
fetch_twist_controller.control_math.transform_twist helper and C++ plugin.
The main algorithm is in step(); ROS setup and message handling follow it.
"""

import threading
import time

import numpy as np
import rospy
import PyKDL as kdl
import tf2_ros
from tf.transformations import quaternion_matrix
from kdl_parser_py.urdf import treeFromParam
from urdf_parser_py.urdf import URDF
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TwistStamped, TransformStamped
from fetch_twist_controller.control_math import transform_twist


class TwistController:
    def __init__(self):
        # Setup happens once. step() below repeats during operation.
        self.lock = threading.Lock()
        self.state = None
        self.command = None
        self.state_received = 0.0
        self.command_received = 0.0

        self.load_parameters()
        self.load_robot_model()
        self.setup_frames()
        self.setup_topics()
        rospy.on_shutdown(self.stop_motion)

    def step(self):
        # STEP 1: MEASURE THE CURRENT JOINT CONFIGURATION.
        # q = measured joint positions; qdot_measured = measured joint speeds.
        # Read in the chain's order, which can include the uncommanded torso.
        state, command, command_received = self.read_fresh_state()
        q, qdot_measured = self.read_joint_values(state)
        q_arm = np.array([q[column] for column in self.arm_columns])

        # STEP 2: COMPUTE THE JACOBIAN AT THE CURRENT CONFIGURATION.
        # J_full maps all chain-joint speeds to tip velocity in root axes.
        # J selects only the seven arm columns that this node commands.
        J_full, tip_pose = self.compute_kinematics(q)
        J = J_full[:, self.arm_columns]

        # Evaluation: calculate ACTUAL twist from measured joint velocities.
        actual_twist = J_full @ qdot_measured
        self.measured_pub.publish(self.twist_message(actual_twist, state.header.stamp))

        # STEP 3: INVERT THE JACOBIAN VELOCITY MAP.
        # Fetch's arm Jacobian is 6x7, so an ordinary matrix inverse won't work.
        # Use a pseudoinverse, with damping near singular configurations.
        J_inverse = self.damped_pseudoinverse(J)

        # STEP 4: MULTIPLY DESIRED TIP TWIST TO GET JOINT VELOCITIES.
        # First express the command in the same axes/reference point as J.
        # A stale or missing twist command becomes six zeros.
        desired_twist = self.command_in_root(command, command_received, state, tip_pose)
        qdot_command = J_inverse @ desired_twist
        qdot_command = self.limit_joint_speeds(qdot_command)
        self.target_pub.publish(self.twist_message(desired_twist, rospy.Time.now()))

        # STEP 5: PREDICT THE JOINT-ANGLE UPDATE: q_next = q + qdot * dt.
        # Here it is a ONE-STEP LIMIT CHECK, not a position command.
        # The Fetch plugin accepts velocities. It does not need q_next.
        # Never replace measured q with this prediction: Step 1 rereads feedback.
        q_next = q_arm + qdot_command * self.dt
        qdot_command = self.limit_next_position(q_arm, q_next, qdot_command)

        # STEP 6: SEND JOINT VELOCITIES AND REPEAT EACH CONTROL CYCLE.
        # The separate C++ plugin receives this message and calls setVelocity().
        self.publish_joint_velocities(qdot_command)

    def run(self):
        # Repeat the six steps at the requested wall-clock rate (default 100 Hz).
        # This is a target frequency, not a hard-real-time scheduling guarantee.
        deadline = time.monotonic()
        while not rospy.is_shutdown():
            try:
                self.step()
            except Exception as error:
                # Bad/missing feedback, a missing TF, or a calculation failure:
                # send zero velocities rather than continuing an old command.
                self.stop_motion()
                rospy.logwarn_throttle(2.0, "Zero output: %s", str(error))

            deadline += self.dt
            remaining = deadline - time.monotonic()
            if remaining > 0.0:
                time.sleep(remaining)
            else:
                rospy.logwarn_throttle(5.0, "Control cycle exceeded %.4f seconds", self.dt)
                deadline = time.monotonic()

    # -------- Helpers for Steps 1 and 2: feedback and kinematics --------

    def read_fresh_state(self):
        with self.lock:
            state = self.state
            command = self.command
            state_received = self.state_received
            command_received = self.command_received

        if state is None:
            raise ValueError("Waiting for /joint_states")
        if not self.is_fresh(state, state_received, self.state_timeout):
            raise ValueError("Joint state is stale or has an invalid timestamp")
        return state, command, command_received

    def is_fresh(self, message, received, timeout):
        # Check both arrival age (wall time) and data age (ROS/simulation time).
        if message is None or message.header.stamp.is_zero():
            return False
        data_age = (rospy.Time.now() - message.header.stamp).to_sec()
        arrival_age = time.monotonic() - received
        return 0.0 <= arrival_age <= timeout and -0.05 <= data_age <= timeout

    def read_joint_values(self, state):
        indices = {name: index for index, name in enumerate(state.name)}
        if len(indices) != len(state.name):
            raise ValueError("Duplicate joint names in feedback")

        q = kdl.JntArray(len(self.chain_names))
        qdot = np.zeros(len(self.chain_names))
        for index, name in enumerate(self.chain_names):
            if name not in indices:
                raise ValueError("Missing joint feedback: " + name)
            source_index = indices[name]
            if source_index >= len(state.position) or source_index >= len(state.velocity):
                raise ValueError("Missing position/velocity feedback: " + name)
            q[index] = state.position[source_index]
            qdot[index] = state.velocity[source_index]

        positions = [q[index] for index in range(q.rows())]
        if not np.all(np.isfinite(positions)) or not np.all(np.isfinite(qdot)):
            raise ValueError("Non-finite joint feedback")
        return q, qdot

    def compute_kinematics(self, q):
        jacobian = kdl.Jacobian(len(self.chain_names))
        tip_pose = kdl.Frame()
        if self.jacobian_solver.JntToJac(q, jacobian) < 0:
            raise ValueError("KDL Jacobian calculation failed")
        if self.forward_solver.JntToCart(q, tip_pose) < 0:
            raise ValueError("KDL forward kinematics failed")

        J = np.array([
            [jacobian[row, col] for col in range(len(self.chain_names))]
            for row in range(6)
        ])
        return J, tip_pose

    # -------- Helpers for Steps 3, 4 and 5: inverse, frames and limits --------

    def damped_pseudoinverse(self, J):
        # SVD breaks J into U, singular values, and V-transpose.
        # Small singular values mean some motion directions are hard to produce.
        U, singular_values, V_transpose = np.linalg.svd(J, full_matrices=False)
        smallest = singular_values[-1]
        damping = self.damping * np.sqrt(
            max(0.0, 1.0 - (smallest / self.threshold) ** 2)
        )

        # Damped reciprocal: s / (s^2 + damping^2).
        denominator = singular_values ** 2 + damping ** 2
        gains = np.divide(
            singular_values,
            denominator,
            out=np.zeros_like(singular_values),
            where=denominator > 1e-16,
        )
        return V_transpose.T @ np.diag(gains) @ U.T

    def command_in_root(self, command, received, state, tip_pose):
        if not self.is_fresh(command, received, self.command_timeout):
            return np.zeros(6)

        twist = command.twist
        requested = np.array([
            twist.linear.x, twist.linear.y, twist.linear.z,
            twist.angular.x, twist.angular.y, twist.angular.z,
        ])
        if not np.all(np.isfinite(requested)):
            raise ValueError("Non-finite twist command")

        frame = command.header.frame_id or rospy.get_param("~default_frame", self.frame)
        tip_position = np.array([tip_pose.p[index] for index in range(3)])

        if frame == self.root:
            rotation = np.eye(3)
            translation = np.zeros(3)
        elif frame == self.tip:
            rotation = np.array([[tip_pose.M[r, c] for c in range(3)] for r in range(3)])
            translation = tip_position
        else:
            # Zero wait: unavailable transforms stop this cycle instead of blocking.
            transform = self.tf_buffer.lookup_transform(
                self.root, frame, state.header.stamp, rospy.Duration(0)
            )
            r = transform.transform.rotation
            p = transform.transform.translation
            rotation = quaternion_matrix([r.x, r.y, r.z, r.w])[:3, :3]
            translation = np.array([p.x, p.y, p.z])

        # Existing helper applies the adjoint and the appropriate point shifts.
        return transform_twist(
            requested, rotation, translation, tip_position, self.reference
        )

    def limit_joint_speeds(self, velocity):
        if not np.all(np.isfinite(velocity)):
            raise ValueError("Non-finite calculated joint velocities")
        # Scale all speeds equally to preserve their relative proportions.
        scale = max(1.0, float(np.max(np.abs(velocity) / self.velocity_limits)))
        return velocity / scale

    def limit_next_position(self, q, q_next, velocity):
        # Nominal one-cycle position bound check; not braking/collision prediction.
        result = velocity.copy()
        for index, bounds in enumerate(self.position_bounds):
            if bounds is None:  # Continuous joints do not have position bounds.
                continue
            lower = bounds[0] + self.margin
            upper = bounds[1] - self.margin
            if result[index] > 0.0 and q_next[index] > upper:
                result[index] = max(0.0, (upper - q[index]) / self.dt)
            elif result[index] < 0.0 and q_next[index] < lower:
                result[index] = min(0.0, (lower - q[index]) / self.dt)
        return result

    # -------- ROS messages: incoming callbacks and outgoing commands --------

    def on_state(self, message):
        with self.lock:
            self.state = message
            self.state_received = time.monotonic()

    def on_command(self, message):
        with self.lock:
            self.command = message
            self.command_received = time.monotonic()

    def publish_joint_velocities(self, velocity):
        message = JointState()
        message.header.stamp = rospy.Time.now()
        message.name = self.names
        message.velocity = list(velocity)
        self.velocity_pub.publish(message)

    def twist_message(self, values, stamp):
        message = TwistStamped()
        message.header.stamp = stamp
        message.header.frame_id = self.root
        message.twist.linear.x, message.twist.linear.y, message.twist.linear.z = values[:3]
        message.twist.angular.x, message.twist.angular.y, message.twist.angular.z = values[3:]
        return message

    def stop_motion(self):
        try:
            self.publish_joint_velocities(np.zeros(len(self.names)))
            # Tell evaluation that the effective target is now zero as well.
            self.target_pub.publish(self.twist_message(np.zeros(6), rospy.Time.now()))
        except rospy.ROSException:
            pass  # Publishing may be unavailable during shutdown; plugin has a watchdog.

    # -------- Setup: runs once before the six-step loop --------

    def load_parameters(self):
        # Define root BEFORE using it as the default for another parameter.
        self.root = rospy.get_param("~root", "base_link")
        self.tip = rospy.get_param("~tip", "gripper_link")
        self.frame = rospy.get_param("~default_frame", self.root)
        self.reference = rospy.get_param("~twist_reference_point", "tip")
        self.rate = float(rospy.get_param("~rate", 100.0))
        self.command_timeout = float(rospy.get_param("~command_timeout", 0.25))
        self.state_timeout = float(rospy.get_param("~state_timeout", 0.2))
        self.damping = float(rospy.get_param("~damping", 0.08))
        self.threshold = float(rospy.get_param("~singularity_threshold", 0.08))
        self.margin = float(rospy.get_param("~position_margin", 0.05))
        self.names = rospy.get_param("~joints")
        self.velocity_limits = np.asarray(rospy.get_param("~velocity_limits"), dtype=float)

        positive = [self.rate, self.command_timeout, self.state_timeout, self.damping, self.threshold]
        if not np.all(np.isfinite(positive)) or min(positive) <= 0.0:
            raise ValueError("Rate, timeouts, damping and threshold must be finite and positive")
        if not self.names or len(set(self.names)) != len(self.names):
            raise ValueError("Joint names must be nonempty and unique")
        if self.velocity_limits.shape != (len(self.names),):
            raise ValueError("Provide one velocity limit per commanded joint")
        if not np.all(np.isfinite(self.velocity_limits)) or np.any(self.velocity_limits <= 0):
            raise ValueError("Velocity limits must be finite and positive")
        if not np.isfinite(self.margin) or self.margin < 0:
            raise ValueError("Position margin must be finite and nonnegative")
        if self.reference not in ("tip", "origin"):
            raise ValueError("twist_reference_point must be tip or origin")
        self.dt = 1.0 / self.rate

    def load_robot_model(self):
        ok, tree = treeFromParam("/robot_description")
        if not ok:
            raise RuntimeError("Cannot parse /robot_description")
        self.chain = tree.getChain(self.root, self.tip)
        self.chain_names = []
        for index in range(self.chain.getNrOfSegments()):
            joint = self.chain.getSegment(index).getJoint()
            if joint.getType() != getattr(kdl.Joint, "None"):
                self.chain_names.append(joint.getName())
        if not set(self.names).issubset(self.chain_names):
            raise ValueError("Commanded joints are not all in the root-to-tip chain")

        self.arm_columns = [self.chain_names.index(name) for name in self.names]
        self.jacobian_solver = kdl.ChainJntToJacSolver(self.chain)
        self.forward_solver = kdl.ChainFkSolverPos_recursive(self.chain)
        model = URDF.from_parameter_server("/robot_description")
        self.position_bounds = []
        for index, name in enumerate(self.names):
            joint = model.joint_map[name]
            if joint.limit and joint.limit.velocity is not None:
                speed_limit = float(joint.limit.velocity)
                if not np.isfinite(speed_limit) or speed_limit <= 0:
                    raise ValueError("Invalid URDF speed limit for " + name)
                self.velocity_limits[index] = min(self.velocity_limits[index], speed_limit)
            if joint.type == "continuous":
                self.position_bounds.append(None)
            else:
                if not joint.limit or joint.limit.lower is None or joint.limit.upper is None:
                    raise ValueError("Missing URDF position bounds for " + name)
                bounds = (joint.limit.lower, joint.limit.upper)
                if not np.all(np.isfinite(bounds)) or bounds[0] + self.margin >= bounds[1] - self.margin:
                    raise ValueError("Invalid position bounds/margin for " + name)
                self.position_bounds.append(bounds)

    def setup_frames(self):
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.broadcaster = tf2_ros.StaticTransformBroadcaster()
        custom = rospy.get_param("~user_frame", {})
        if not custom:
            return
        xyz = np.asarray(custom["translation"], dtype=float)
        quaternion = np.asarray(custom["quaternion"], dtype=float)
        if xyz.shape != (3,) or quaternion.shape != (4,):
            raise ValueError("User frame needs xyz[3] and quaternion[4]")
        if not np.all(np.isfinite(xyz)) or not np.all(np.isfinite(quaternion)):
            raise ValueError("User frame contains non-finite values")
        if np.linalg.norm(quaternion) < 1e-8:
            raise ValueError("User-frame quaternion cannot be zero")
        quaternion = quaternion / np.linalg.norm(quaternion)
        message = TransformStamped()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = custom["parent"]
        message.child_frame_id = custom["name"]
        message.transform.translation.x, message.transform.translation.y, message.transform.translation.z = xyz
        message.transform.rotation.x, message.transform.rotation.y, message.transform.rotation.z, message.transform.rotation.w = quaternion
        self.broadcaster.sendTransform(message)

    def setup_topics(self):
        self.velocity_pub = rospy.Publisher("/fetch_joint_velocity/command", JointState, queue_size=1)
        self.target_pub = rospy.Publisher("~target_twist", TwistStamped, queue_size=20)
        self.measured_pub = rospy.Publisher("~measured_twist", TwistStamped, queue_size=20)
        rospy.Subscriber("/joint_states", JointState, self.on_state, queue_size=1)
        rospy.Subscriber("/fetch_twist/command", TwistStamped, self.on_command, queue_size=1)
        rospy.loginfo("Twist controller: %s -> %s, %.1f Hz", self.root, self.tip, self.rate)


if __name__ == "__main__":
    rospy.init_node("twist_controller")
    TwistController().run()
