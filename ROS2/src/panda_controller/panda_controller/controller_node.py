import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import numpy as np
import roboticstoolbox as rtb # Python version of the Peter-Corke MATLAB package

# Joint names as defined in the URDF (fer = Franka Emika Research) downloaded from the ROS Humble package
JOINT_NAMES = ['fer_joint1', 'fer_joint2', 'fer_joint3', 'fer_joint4',
               'fer_joint5', 'fer_joint6', 'fer_joint7']

# Initial joint configuration: initial values defined in the URDF file
Q0 = np.array([0.0, 0.0, 0.0, -np.pi/2, 0.0, np.pi/2, np.pi/4])


class PandaController(Node):
    def __init__(self):
        super().__init__('panda_controller')

        # Load Panda robot model
        self.panda = rtb.models.Panda()

        # Controller parameters (from MATLAB)
        self.mode   = 3          # 0=CLIK only, 1=joint limits, 2=manipulability, 3=both
        self.Ts     = 1e-3       # Integration step [s] 
        self.T_end  = 1.0        # Trajectory duration [s]
        self.r      = 0.4        # Circle radius [m]
        self.omega  = 2 * np.pi  # Angular velocity [rad/s]
        self.K_p    = 10.0 * np.eye(3)   # Position error gain
        self.K_o    = 10.0 * np.eye(3)   # Orientation error gain
        self.lam    = 0.01       # Damping factor for DLS pseudoinverse
        self.k0     = 375     # Null-space gain for joint limit avoidance
        self.k1     = 250      # Null-space gain for manipulability maximisation
        self.delta  = 1e-3      # Finite-difference step for manipulability gradient

        # Joint limits from robot model
        q_min = self.panda.qlim[0, :]
        q_max = self.panda.qlim[1, :]
        self.q_mid   = (q_max + q_min) / 2   # Joint midpoint
        self.q_range = q_max - q_min         # Joint range

        # Creating variables for final metrics summary
        self.err_hist   = []
        self.w_hist     = []
        self.qdot_hist  = []
        self.dist_hist       = []
        self.violation_count = 0
        self.metrics_printed = False

        # CLOSED-LOOP SYSTEM
        # q is integrated in the control loop and sent as position command to ros2_control.
        # ros2_control return q back on /joint_states, to which the node is subscribed, closing the loop.

        # Initial configuration
        self.q = Q0.copy()
        T0 = self.panda.fkine(self.q)
        self.eePos_i = T0.t.copy()   # Initial end-effector position 
        self.Rdes    = T0.R.copy()   # Desired orientation (kept constant)

        # Step counter — identical to MATLAB: t advances by Ts each step
        self.t = 0.0
        self.log_counter  = 0 # To not overload the logging window in the terminal (log every 100 steps)
        self.path_counter = 0 # Publish position for path visualization every 10 steps

        # Feedback from ros2_control via joint_state_broadcaster.
        # Until the first /joint_states message arrives, the controller uses self.q (internal).
        self.q_feedback     = None
        self.feedback_ready = False

        # Publisher 1: Integrated joint positions → ros2_control position interface
        self.cmd_pub = self.create_publisher(Float64MultiArray, '/position_controller/commands', 10)
        # Publisher 2: End-effector path for visualisation in RViz
        self.path_pub = self.create_publisher(Path, '/ee_path', 10)
        self.ee_path_msg = Path()
        self.ee_path_msg.header.frame_id = 'fer_link0' # Specifying the reference frame

        # Subscriber: Joint positions returned exactly as we computed from joint_state_broadcaster (ros2_control output)
        self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)

        # Print initial state
        J0 = self.panda.jacob0(self.q)
        w0 = np.sqrt(max(0.0, np.linalg.det(J0 @ J0.T)))
        self.get_logger().info(
            f'q0={np.round(self.q, 3).tolist()} | '
            f'End-effector starting position=({T0.t[0]:.4f},{T0.t[1]:.4f},{T0.t[2]:.4f}) | Manipulability={w0:.4f}'
        )

        # Main control loop at 1 kHz
        self.create_timer(self.Ts, self.control_loop)

    def joint_state_callback(self, msg):
        #Receive joint positions from ros2_control and store as feedback (activated only when ros2_control resends back the position)
        pos = dict(zip(msg.name, msg.position)) # Saving a dictionary (joint_name, joint_position)
        self.q_feedback = np.array([pos[j] for j in JOINT_NAMES])
        if not self.feedback_ready:
            self.feedback_ready = True # If we have received the feedback from ros2_control
            self.get_logger().info('Closed-loop active: Reading q from /joint_states.')

    def control_loop(self):
        # Main CLIK control loop from MATLAB

        # Step counter: t advances by Ts each call (identical to MATLAB)
        t = self.t
        self.t += self.Ts

        # Stop sending new commands when we reach T_end and print metrics summary
        if t > self.T_end:
            if not self.metrics_printed:
                self.metrics_printed = True # Stop the printing even if the controller is still going

                err = np.array(self.err_hist)
                self.get_logger().info(
                    f'\n--- Mode {self.mode} ---\n'
                    f'Max tracking error:  {np.max(err)*1000:.2f} mm\n'
                    f'RMS tracking error:  {np.sqrt(np.mean(err**2))*1000:.2f} mm\n'
                    f'Mean manipulability: {np.mean(self.w_hist):.4f}\n'
                    f'Min manipulability:  {np.min(self.w_hist):.4f}\n'
                    f'Max joint velocity:  {np.max(self.qdot_hist):.2f} rad/s\n'
                    f'Max dist from joint center: {np.max(self.dist_hist):.4f} rad\n'
                    f'Mean dist from joint center: {np.mean(self.dist_hist):.4f} rad\n'
                    f'Joint limit violations: {self.violation_count} / {len(self.err_hist)}'
                )
            cmd = Float64MultiArray()
            cmd.data = self.q.tolist()
            self.cmd_pub.publish(cmd)
            return
                

        # Use feedback from ros2_control when available, otherwise keep the internal state
        if self.feedback_ready:
            q = self.q_feedback.copy() 
        else:
            q = self.q.copy()

        # Forward kinematics and Jacobian at current configuration
        T_curr   = self.panda.fkine(q)
        eeP_curr = T_curr.t
        J_curr = self.panda.jacob0(q)

        # Desired trajectory
        xd     = self.eePos_i[0] + self.r * np.cos(self.omega * t) - self.r
        yd     = self.eePos_i[1] + self.r * np.sin(self.omega * t)
        xd_dot = -self.r * self.omega * np.sin(self.omega * t)
        yd_dot =  self.r * self.omega * np.cos(self.omega * t)
        pos_des = np.array([xd, yd, self.eePos_i[2]])

        # Position error
        err_p = pos_des - eeP_curr
        # Orientation error (rotation matrix formulation)
        Re    = self.Rdes @ T_curr.R.T
        err_o = 0.5 * np.array([Re[2,1]-Re[1,2], Re[0,2]-Re[2,0], Re[1,0]-Re[0,1]])

        # Task-space velocity: feedforward + proportional correction
        vel_ctrl = np.concatenate([
            np.array([xd_dot, yd_dot, 0.0]) + self.K_p @ err_p,
            self.K_o @ err_o
        ])

        # Damped Least Squares (DLS) pseudoinverse 
        w = np.sqrt(max(0.0, np.linalg.det(J_curr @ J_curr.T)))   # Manipulability for logging
        pseudoInv = J_curr.T @ np.linalg.inv(J_curr @ J_curr.T + self.lam**2 * np.eye(6))

        # Null-space projector
        N = np.eye(7) - pseudoInv @ J_curr
        # Null-space task initialized
        null_task = np.zeros(7)

        # Mode 1 or 3: joint limit avoidance (gradient of distance from midpoint)
        if self.mode in (1, 3):
            null_task += self.k0 * (-(q - self.q_mid) / self.q_range**2)

        # Mode 2 or 3: manipulability maximisation (numerical gradient of w)
        if self.mode in (2, 3):
            grad_w = np.zeros(7)
            for i in range(7):
                q_plus = q.copy()
                q_plus[i] += self.delta
                q_minus = q.copy()
                q_minus[i] -= self.delta
                J_plus = self.panda.jacob0(q_plus)
                J_minus = self.panda.jacob0(q_minus)
                w_plus = np.sqrt(np.linalg.det(J_plus @ J_plus.T))
                w_minus = np.sqrt(np.linalg.det(J_minus @ J_minus.T))
                grad_w[i] = (w_plus - w_minus) / (2 * self.delta)
            null_task += self.k1 * grad_w

        # Joint velocity and Euler integration
        # q_dot = J^† v + N * null_task
        q_dot  = pseudoInv @ vel_ctrl + N @ null_task
        self.q = self.q + q_dot * self.Ts

        # Saving current data
        self.err_hist.append(np.linalg.norm(err_p))
        self.w_hist.append(w)
        self.qdot_hist.append(np.max(np.abs(q_dot)))
        self.dist_hist.append(np.max(np.abs(q - self.q_mid)))
        violations = int(np.any((q < self.panda.qlim[0,:]) | (q > self.panda.qlim[1,:])))
        self.violation_count += violations

        # Send integrated position to ros2_control
        cmd = Float64MultiArray()
        cmd.data = self.q.tolist()
        self.cmd_pub.publish(cmd)

        # Logging every 100 steps (every 100 ms)
        self.log_counter += 1
        if self.log_counter % 100 == 0:
            phase = 'HOLD' if t < 0.0 else f't={t:.2f}s'
            self.get_logger().info(
                f'{phase} | Tracking error={np.linalg.norm(err_p)*1000:.1f}mm | '
                f'Max joint velocity={np.max(np.abs(q_dot)):.2f} rad/s | Manipulability={w:.4f}'
            )

        # Publish end-effector path for RViz (every 10 steps to reduce overhead)
        self.path_counter += 1
        if self.path_counter % 10 == 0:
            stamp = self.get_clock().now().to_msg()
            pose = PoseStamped()
            pose.header.stamp = stamp
            pose.header.frame_id = 'fer_link0'
            pose.pose.position.x = float(eeP_curr[0])
            pose.pose.position.y = float(eeP_curr[1])
            pose.pose.position.z = float(eeP_curr[2])
            pose.pose.orientation.w = 1.0
            self.ee_path_msg.header.stamp = stamp
            self.ee_path_msg.poses.append(pose)
            self.path_pub.publish(self.ee_path_msg)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(PandaController())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
