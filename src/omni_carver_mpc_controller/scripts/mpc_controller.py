# mpc_controller.py

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Path, OccupancyGrid
import tf2_ros
import numpy as np
from scipy.optimize import minimize


class MpcController(Node):
    def __init__(self):
        super().__init__('mpc_controller')

        # ─── PARAMETERS ────────────────────────────────────────────────────────────
        self.declare_parameter('prediction_horizon',     10)
        self.declare_parameter('dt',                    0.2)
        self.declare_parameter('w_trans',               1.0)
        self.declare_parameter('w_orient',              1.0)
        self.declare_parameter('w_control',             0.01)
        self.declare_parameter('orient_activation_dist',0.5)
        self.declare_parameter('costmap_hard_threshold',0.8)
        self.declare_parameter('low_pass_gain',         0.5)
        self.declare_parameter('lookahead_dist_max',    1.0)

        # fetch parameters
        self.N         = self.get_parameter('prediction_horizon').value
        self.dt        = self.get_parameter('dt').value
        self.Q_trans   = self.get_parameter('w_trans').value
        self.Q_orient  = self.get_parameter('w_orient').value
        self.R         = self.get_parameter('w_control').value
        self.orient_activation_dist = self.get_parameter('orient_activation_dist').value
        self.cost_threshold= self.get_parameter('costmap_hard_threshold').value
        self.alpha     = self.get_parameter('low_pass_gain').value
        self.ld_max    = self.get_parameter('lookahead_dist_max').value

        # ─── STATE ────────────────────────────────────────────────────────────────
        self.current_path = []
        self.prev_u       = np.zeros((3,))  # [vx, vy, ω]
        self.last_path_id = None
        self.costmap      = None

        # ─── ROS COMM ─────────────────────────────────────────────────────────────
        self.path_sub    = self.create_subscription(Path, 'global_path',
                                                    self.path_callback, 10)
        self.cmd_pub     = self.create_publisher(TwistStamped, 'cmd_vel', 10)
        self.costmap_sub = self.create_subscription(
            OccupancyGrid,
            '/local_costmap/costmap',
            self.costmap_callback,
            1)

        # ─── TF ───────────────────────────────────────────────────────────────────
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ─── CONTROL LOOP ─────────────────────────────────────────────────────────
        self.timer = self.create_timer(self.dt, self.control_loop)

    def path_callback(self, msg: Path):
        # detect replanning & reset filter
        path_id = tuple((p.pose.position.x, p.pose.position.y)
                        for p in msg.poses[:3])
        if path_id != self.last_path_id:
            self.prev_u = np.zeros((3,))
            self.last_path_id = path_id
        self.current_path = msg.poses

    def costmap_callback(self, msg: OccupancyGrid):
        self.costmap = {
            'data': np.array(msg.data, np.uint8).reshape((msg.info.height, msg.info.width)),
            'origin_x': msg.info.origin.position.x,
            'origin_y': msg.info.origin.position.y,
            'resolution': msg.info.resolution,
            'width': msg.info.width,
            'height': msg.info.height
        }

    def control_loop(self):
        if not self.current_path or self.costmap is None:
            return

        # a) get current pose
        x0, theta0 = self.get_current_pose()

        # b) select lookahead segment
        segment = self.select_lookahead(self.current_path, x0, self.ld_max)

        # c) build reference and distances
        X_ref, dists = self.build_reference(segment)

        # d) linearize & discretize
        A, B = self.linearize_and_discretize(theta0, self.dt)

        # e) build cost matrices
        Q_block, R_block = self.build_cost_matrices(X_ref, dists)

        # f) sample costmap & hard-stop
        cost_penalty, max_cost = self.sample_costmap_and_penalty(X_ref)
        if max_cost >= self.cost_threshold:
            u0 = np.zeros((3,))
            self.prev_u = u0
            return self.publish_command(u0)

        # g) build prediction matrices
        A_block, B_block = self.build_prediction_matrices(A, B)

        # h) solve QP
        U_opt = self.solve_qp_scipy(A_block, B_block, x0, X_ref, Q_block, R_block)

        # i) low-pass filter & first control
        u0 = self.alpha * U_opt[0] + (1 - self.alpha) * self.prev_u
        self.prev_u = u0

        # j) publish
        self.publish_command(u0)

    def get_current_pose(self):
        t = self.tf_buffer.lookup_transform('odom', 'base_link', rclpy.time.Time())
        x = t.transform.translation.x
        y = t.transform.translation.y
        q = t.transform.rotation
        theta = np.arctan2(2*(q.w*q.z + q.x*q.y),
                           1 - 2*(q.y*q.y + q.z*q.z))
        return np.array([x, y]), theta

    def select_lookahead(self, path, x0, d_max):
        pts = []
        for pose in path:
            stamped = pose
            stamped.header.frame_id = pose.header.frame_id
            try:
                tf_pose = self.tf_buffer.transform(stamped, 'base_link', timeout=Duration(seconds=0.1))
                dx = tf_pose.pose.position.x
                dy = tf_pose.pose.position.y
                dist = np.hypot(dx, dy)
                if dist <= d_max:
                    pts.append((pose, dist))
            except Exception:
                continue

        if len(pts) > self.N:
            idxs = np.linspace(0, len(pts)-1, self.N, dtype=int)
            selected = [pts[i][0] for i in idxs]
        else:
            selected = [p for p, _ in pts]
            while len(selected) < self.N:
                selected.append(selected[-1])
        return selected

    def build_reference(self, segment):
        X_ref = np.zeros((self.N, 3))
        dists = np.zeros(self.N)
        x0 = self.get_current_pose()[0]
        for i, pose in enumerate(segment):
            x = pose.pose.position.x
            y = pose.pose.position.y
            q = pose.pose.orientation
            theta = np.arctan2(2*(q.w*q.z + q.x*q.y),
                               1 - 2*(q.y*q.y + q.z*q.z))
            X_ref[i] = [x, y, theta]
            dists[i] = np.linalg.norm([x - x0[0], y - x0[1]])
        return X_ref, dists

    def linearize_and_discretize(self, theta, dt):
        I = np.eye(3)
        B = dt * np.array([[ np.cos(theta), -np.sin(theta), 0],
                           [ np.sin(theta),  np.cos(theta), 0],
                           [             0,              0, 1]])
        return I, B

    def build_prediction_matrices(self, A, B):
        A_block = np.zeros((3*self.N, 3))
        B_block = np.zeros((3*self.N, 3*self.N))
        for i in range(self.N):
            A_block[3*i:3*(i+1), :] = np.linalg.matrix_power(A, i+1)
            for j in range(i+1):
                B_block[3*i:3*(i+1), 3*j:3*(j+1)] = np.linalg.matrix_power(A, i-j) @ B
        return A_block, B_block

    def build_cost_matrices(self, X_ref, dists):
        Q_blocks = []
        for d in dists:
            w_or = self.Q_orient if d <= self.orient_activation_dist else 0.0
            Q_blocks.append(np.diag([self.Q_trans, self.Q_trans, w_or]))
        Q_block = np.block([[Q_blocks[i] if i == j else np.zeros((3,3))
                              for j in range(self.N)] for i in range(self.N)])
        R_block = self.R * np.eye(3 * self.N)
        return Q_block, R_block

    def sample_costmap_and_penalty(self, X_ref):
        cm = self.costmap
        pen = []
        maxc = 0.0
        for x, y, _ in X_ref:
            ix = int((x - cm['origin_x']) / cm['resolution'])
            iy = int((y - cm['origin_y']) / cm['resolution'])
            if 0 <= ix < cm['width'] and 0 <= iy < cm['height']:
                c = cm['data'][iy, ix] / 255.0
            else:
                c = 1.0
            pen.append(c)
            maxc = max(maxc, c)
        return np.array(pen), maxc

    def solve_qp_scipy(self, A_block, B_block, x0, X_ref, Q, R):
        # build free-run state deviations
        x0_flat = A_block @ x0
        X_ref_flat = X_ref.flatten()
        H = B_block.T @ Q @ B_block + R
        g = (x0_flat - X_ref_flat).T @ Q @ B_block

        def obj(U):
            return 0.5 * U @ H @ U + g @ U

        U0 = np.zeros(self.N * 3)
        res = minimize(obj, U0, method='SLSQP',
                       options={'maxiter': 50, 'ftol': 1e-3})
        return res.x.reshape((self.N, 3))

    def publish_command(self, u):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.linear.x  = float(u[0])
        msg.twist.linear.y  = float(u[1])
        msg.twist.angular.z = float(u[2])
        self.cmd_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MpcController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()