#!/usr/bin/env python3
import math
from time import time as wall_time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2 as pc2
from nav_msgs.msg import OccupancyGrid
from tf2_ros import Buffer, TransformListener
from rclpy.duration import Duration
from rclpy.time import Time

import torch
import gpytorch
import gpytorch.settings as gps
from gpytorch.means import ConstantMean
from gpytorch.kernels import ScaleKernel, RBFKernel, InducingPointKernel
from gpytorch.distributions import MultivariateNormal

try:
    from scipy.ndimage import maximum_filter, minimum_filter, gaussian_filter
    SCIPY_OK = True
except Exception:
    SCIPY_OK = False


def quat_to_rot_matrix(qx, qy, qz, qw):
    """Quaternion -> 3x3 rotation matrix."""
    xx = qx * qx
    yy = qy * qy
    zz = qz * qz
    xy = qx * qy
    xz = qx * qz
    yz = qy * qz
    wx = qw * qx
    wy = qw * qy
    wz = qw * qz

    return np.array([
        [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz),       2.0 * (xz + wy)],
        [2.0 * (xy + wz),       1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
        [2.0 * (xz - wy),       2.0 * (yz + wx),       1.0 - 2.0 * (xx + yy)],
    ], dtype=np.float32)


def fit_plane_rms(window_z, window_x, window_y, valid_mask):
    """Least-squares plane residual RMS over one local window."""
    idx = np.where(valid_mask)
    if idx[0].size < 6:
        return np.nan

    x = window_x[idx].reshape(-1, 1)
    y = window_y[idx].reshape(-1, 1)
    z = window_z[idx].reshape(-1, 1)

    A = np.hstack([x, y, np.ones_like(x)])
    try:
        coeff, *_ = np.linalg.lstsq(A, z, rcond=None)
        z_fit = A @ coeff
        rms = np.sqrt(np.mean((z - z_fit) ** 2))
        return float(rms)
    except Exception:
        return np.nan


class SGPModel(gpytorch.models.ExactGP):
    def __init__(self, train_x, train_y, likelihood, inducing_points_count, lengthscale_xy, outputscale):
        super().__init__(train_x, train_y, likelihood)
        self.mean_module = ConstantMean()

        n = train_x.shape[0]
        m = min(max(16, inducing_points_count), n)
        step = max(1, n // m)
        inducing_idx = np.arange(0, n, step, dtype=np.int32)[:m]
        inducing_variable = train_x[inducing_idx, :]

        base_kernel = ScaleKernel(
            RBFKernel(ard_num_dims=2)
        )
        base_kernel.outputscale = outputscale
        base_kernel.base_kernel.lengthscale = torch.tensor(
            [lengthscale_xy, lengthscale_xy],
            dtype=torch.float32,
            device=train_x.device
        )

        self.covar_module = InducingPointKernel(
            base_kernel,
            inducing_points=inducing_variable,
            likelihood=likelihood
        )

    def forward(self, x):
        mean_x = self.mean_module(x)
        covar_x = self.covar_module(x)
        return MultivariateNormal(mean_x, covar_x)


class GPMappingModuleNode(Node):
    def __init__(self):
        super().__init__('gp_mapping_module')

        # Topics / frames
        self.declare_parameter('cloud_topic', '/cloud_registered_body')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'body')
        self.declare_parameter('publish_frame', 'odom')

        # Local map geometry
        self.declare_parameter('resolution', 0.2)
        self.declare_parameter('length_in_x', 10.0)
        self.declare_parameter('length_in_y', 10.0)
        self.declare_parameter('max_range_m', 4.0)
        self.declare_parameter('min_range_m', 0.15)
        self.declare_parameter('max_points', 8000)

        # Vertical filtering after transform to odom/world
        self.declare_parameter('z_min_world', -2.00)
        self.declare_parameter('z_max_world', 2.0)

        # Self-filter / mast wedge
        self.declare_parameter('fov_deg', 360.0)
        self.declare_parameter('pole_center_deg', 0.0)
        self.declare_parameter('pole_width_deg', 0.0)
        self.declare_parameter('self_radius_m', 0.22)

        # GP params
        self.declare_parameter('inducing_points', 500)
        self.declare_parameter('gp_noise', 0.01)
        self.declare_parameter('kernel_lengthscale', 0.25)
        self.declare_parameter('kernel_outputscale', 0.15)

        # Rolling window
        self.declare_parameter('origin_hold_m', 0.10)

        # Traversability thresholds
        self.declare_parameter('slope_crit', 0.45)        # gradient magnitude threshold
        self.declare_parameter('flatness_crit', 0.06)     # RMS plane residual (m)
        self.declare_parameter('step_crit', 0.12)         # step / drop threshold (m)
        self.declare_parameter('sigma_percentile', 98.0)  # unknown mask percentile

        # Traversability weights
        self.declare_parameter('w_slope', 0.40)
        self.declare_parameter('w_flatness', 0.25)
        self.declare_parameter('w_step', 0.35)

        # Footprint window sizes
        self.declare_parameter('flatness_window_m', 0.35)
        self.declare_parameter('step_window_m', 0.25)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.get_logger().info(f'Using device: {self.device}')

        self._last_center = None

        # Publishers
        self.pub_elev = self.create_publisher(PointCloud2, '/elevation_pcl', 1)
        self.pub_slope = self.create_publisher(PointCloud2, '/gp_slope', 1)
        self.pub_unc = self.create_publisher(PointCloud2, '/uncertainty', 1)

        qos_costmap = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.pub_costmap = self.create_publisher(OccupancyGrid, '/gp_costmap', qos_costmap)

        self.fields_out = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]

        cloud_topic = self.get_parameter('cloud_topic').value
        self.sub = self.create_subscription(
            PointCloud2,
            cloud_topic,
            self.cloud_cb,
            qos_profile_sensor_data
        )

        self.get_logger().info(f'Subscribed to: {cloud_topic}')
        self.get_logger().info('Publishing: /elevation_pcl, /gp_slope, /uncertainty, /gp_costmap')

    def sampling_grid(self, resolution, x_length, y_length):
        x_range = x_length / 2.0
        y_range = y_length / 2.0

        x_s = np.arange(-x_range, x_range, resolution, dtype=np.float32)
        y_s = np.arange(-y_range, y_range, resolution, dtype=np.float32)

        X, Y = np.meshgrid(x_s, y_s)
        grid_local = np.stack([X.ravel(), Y.ravel()], axis=1).astype(np.float32)
        return grid_local, x_s, y_s

    def transform_points(self, pts_xyz, tf_msg):
        """Apply full 3D transform to Nx3 points."""
        t = tf_msg.transform.translation
        q = tf_msg.transform.rotation
        R = quat_to_rot_matrix(q.x, q.y, q.z, q.w)
        trans = np.array([t.x, t.y, t.z], dtype=np.float32)
        return (pts_xyz @ R.T) + trans

    def get_consistent_tf(self, target_frame, source_frame, msg_stamp):
        try:
            if self.tf_buffer.can_transform(
                target_frame,
                source_frame,
                msg_stamp,
                timeout=Duration(seconds=0.03)
            ):
                return self.tf_buffer.lookup_transform(target_frame, source_frame, msg_stamp)
            return self.tf_buffer.lookup_transform(target_frame, source_frame, Time(), timeout=Duration(seconds=0.03))
        except Exception:
            return None

    def publish_cloud(self, header, xyz, intensity):
        pts = np.hstack([xyz[:, 0:1], xyz[:, 1:2], xyz[:, 2:3], intensity.reshape(-1, 1)]).astype(np.float32)
        return pc2.create_cloud(header, self.fields_out, pts.tolist())

    def cloud_cb(self, msg: PointCloud2):
        t0 = wall_time()

        odom_frame = self.get_parameter('odom_frame').value
        base_frame = self.get_parameter('base_frame').value
        publish_frame = self.get_parameter('publish_frame').value

        stamp = Time.from_msg(msg.header.stamp)
        tf = self.get_consistent_tf(odom_frame, base_frame, stamp)
        if tf is None:
            self.get_logger().warn(f'TF lookup failed {odom_frame} <- {base_frame}')
            return

        # Parse cloud
        try:
            pts = []
            for p in pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True):
                pts.append([float(p[0]), float(p[1]), float(p[2])])
            pcl_body = np.array(pts, dtype=np.float32)
        except Exception as e:
            self.get_logger().error(f'PointCloud2 parse failed: {e}')
            return

        if pcl_body.shape[0] < 50:
            return

        # Body-frame radial / FOV / self filtering before transform
        x_b = pcl_body[:, 0]
        y_b = pcl_body[:, 1]
        z_b = pcl_body[:, 2]

        r = np.sqrt(x_b * x_b + y_b * y_b + z_b * z_b)
        min_range = float(self.get_parameter('min_range_m').value)
        max_range = float(self.get_parameter('max_range_m').value)
        keep = (r >= min_range) & (r <= max_range)

        # Remove very near self returns using XY radius
        self_radius = float(self.get_parameter('self_radius_m').value)
        r_xy = np.sqrt(x_b * x_b + y_b * y_b)
        keep &= (r_xy >= self_radius)

        # FOV + pole wedge in body frame
        fov_deg = float(self.get_parameter('fov_deg').value)
        pole_center_deg = float(self.get_parameter('pole_center_deg').value)
        pole_width_deg = float(self.get_parameter('pole_width_deg').value)

        ang = np.arctan2(y_b, x_b)
        if fov_deg < 360.0:
            half = np.deg2rad(fov_deg) / 2.0
            keep &= (np.abs(ang) <= half)

        if pole_width_deg > 0.0:
            c0 = np.deg2rad(pole_center_deg)
            w = np.deg2rad(pole_width_deg) / 2.0
            dang = np.arctan2(np.sin(ang - c0), np.cos(ang - c0))
            keep &= (np.abs(dang) > w)

        pcl_body = pcl_body[keep]
        if pcl_body.shape[0] < 50:
            return

        # Full 3D transform into odom/world-consistent frame
        pcl_odom = self.transform_points(pcl_body[:, :3], tf)

        # World-frame vertical filter
        z_min = float(self.get_parameter('z_min_world').value)
        z_max = float(self.get_parameter('z_max_world').value)
        keep_z = (pcl_odom[:, 2] >= z_min) & (pcl_odom[:, 2] <= z_max)
        pcl_odom = pcl_odom[keep_z]
        if pcl_odom.shape[0] < 50:
            return

        # Window center from transform
        tx = tf.transform.translation.x
        ty = tf.transform.translation.y

        hold = float(self.get_parameter('origin_hold_m').value)
        if self._last_center is None:
            self._last_center = (float(tx), float(ty))
        else:
            lx, ly = self._last_center
            dx = float(tx) - lx
            dy = float(ty) - ly
            if (dx * dx + dy * dy) < (hold * hold):
                tx, ty = lx, ly
            else:
                self._last_center = (float(tx), float(ty))

        # Crop to local rolling window in odom
        res = float(self.get_parameter('resolution').value)
        x_len = float(self.get_parameter('length_in_x').value)
        y_len = float(self.get_parameter('length_in_y').value)

        x_min = tx - x_len / 2.0
        x_max = tx + x_len / 2.0
        y_min = ty - y_len / 2.0
        y_max = ty + y_len / 2.0

        keep_win = (
            (pcl_odom[:, 0] >= x_min) & (pcl_odom[:, 0] <= x_max) &
            (pcl_odom[:, 1] >= y_min) & (pcl_odom[:, 1] <= y_max)
        )
        pcl_odom = pcl_odom[keep_win]
        if pcl_odom.shape[0] < 50:
            return

        # Downsample by stride if needed
        max_points = int(self.get_parameter('max_points').value)
        if pcl_odom.shape[0] > max_points:
            step = max(1, pcl_odom.shape[0] // max_points)
            pcl_odom = pcl_odom[::step][:max_points]

        # Prepare training data
        d_in = pcl_odom[:, :2].astype(np.float32)
        d_out = pcl_odom[:, 2].astype(np.float32)

        # Quantized dedup
        key = np.round(d_in / res).astype(np.int32)
        _, unique_idx = np.unique(key, axis=0, return_index=True)
        d_in = d_in[unique_idx]
        d_out = d_out[unique_idx]

        if d_in.shape[0] < 40:
            return

        order = np.lexsort((d_in[:, 1], d_in[:, 0]))
        d_in = d_in[order]
        d_out = d_out[order]

        train_x = torch.tensor(d_in, dtype=torch.float32, device=self.device)
        train_y = torch.tensor(d_out, dtype=torch.float32, device=self.device)

        # GP setup
        gp_noise = float(self.get_parameter('gp_noise').value)
        inducing_points = int(self.get_parameter('inducing_points').value)
        kernel_lengthscale = float(self.get_parameter('kernel_lengthscale').value)
        kernel_outputscale = float(self.get_parameter('kernel_outputscale').value)

        likelihood = gpytorch.likelihoods.GaussianLikelihood()
        likelihood.noise = torch.tensor(gp_noise, dtype=torch.float32, device=self.device)
        likelihood.raw_noise.requires_grad_(False)

        model = SGPModel(
            train_x=train_x,
            train_y=train_y,
            likelihood=likelihood,
            inducing_points_count=inducing_points,
            lengthscale_xy=kernel_lengthscale,
            outputscale=kernel_outputscale
        ).to(self.device)

        model.eval()
        likelihood.eval()

        # Query grid in odom
        grid_local, x_s, y_s = self.sampling_grid(res, x_len, y_len)
        W = len(x_s)
        H = len(y_s)

        grid = grid_local.copy()
        grid[:, 0] += float(tx)
        grid[:, 1] += float(ty)

        Xtest = torch.tensor(grid, dtype=torch.float32, device=self.device, requires_grad=True)

        with gps.cholesky_jitter(1e-3):
            with torch.autograd.set_grad_enabled(True):
                preds = likelihood(model(Xtest))

        mean = preds.mean.detach().cpu().numpy().reshape(H, W)
        var = preds.variance.detach().cpu().numpy().reshape(H, W)

        ones = torch.ones_like(preds.mean)
        grad_mean = torch.autograd.grad(preds.mean, Xtest, grad_outputs=ones, retain_graph=False)[0]
        grad_xy = grad_mean.detach().cpu().numpy().reshape(H, W, 2)
        slope = np.sqrt(grad_xy[:, :, 0] ** 2 + grad_xy[:, :, 1] ** 2)

        # Validity based on proximity to observed region
        obs_mask = np.zeros((H, W), dtype=bool)
        obs_ix = np.clip(np.floor((d_in[:, 0] - x_min) / res).astype(np.int32), 0, W - 1)
        obs_iy = np.clip(np.floor((d_in[:, 1] - y_min) / res).astype(np.int32), 0, H - 1)
        obs_mask[obs_iy, obs_ix] = True

        # Expand observed support slightly
        if SCIPY_OK:
            obs_mask_f = maximum_filter(obs_mask.astype(np.uint8), size=3) > 0
        else:
            obs_mask_f = obs_mask.copy()

        # Step-height map: local max-min in footprint window
        step_window_m = float(self.get_parameter('step_window_m').value)
        step_k = max(1, int(round(step_window_m / res)))
        if step_k % 2 == 0:
            step_k += 1

        if SCIPY_OK:
            local_max = maximum_filter(mean, size=step_k, mode='nearest')
            local_min = minimum_filter(mean, size=step_k, mode='nearest')
            step_map = np.maximum(np.abs(local_max - mean), np.abs(mean - local_min))
        else:
            step_map = np.zeros_like(mean, dtype=np.float32)

        # Flatness map: plane-fit RMS in footprint window
        flatness_window_m = float(self.get_parameter('flatness_window_m').value)
        flat_k = max(3, int(round(flatness_window_m / res)))
        if flat_k % 2 == 0:
            flat_k += 1

        Xg = grid[:, 0].reshape(H, W)
        Yg = grid[:, 1].reshape(H, W)

        flatness = np.full((H, W), np.nan, dtype=np.float32)
        pad = flat_k // 2
        mean_pad = np.pad(mean, pad, mode='edge')
        X_pad = np.pad(Xg, pad, mode='edge')
        Y_pad = np.pad(Yg, pad, mode='edge')
        valid_pad = np.pad(obs_mask_f.astype(np.uint8), pad, mode='edge').astype(bool)

        for j in range(H):
            for i in range(W):
                wz = mean_pad[j:j + flat_k, i:i + flat_k]
                wx = X_pad[j:j + flat_k, i:i + flat_k]
                wy = Y_pad[j:j + flat_k, i:i + flat_k]
                wm = valid_pad[j:j + flat_k, i:i + flat_k]
                flatness[j, i] = fit_plane_rms(wz, wx, wy, wm)

        flatness = np.nan_to_num(flatness, nan=np.nanmax(flatness[np.isfinite(flatness)]) if np.any(np.isfinite(flatness)) else 0.0)

        # Normalize traversability components
        slope_crit = float(self.get_parameter('slope_crit').value)
        flatness_crit = float(self.get_parameter('flatness_crit').value)
        step_crit = float(self.get_parameter('step_crit').value)

        w_slope = float(self.get_parameter('w_slope').value)
        w_flat = float(self.get_parameter('w_flatness').value)
        w_step = float(self.get_parameter('w_step').value)
        w_sum = max(w_slope + w_flat + w_step, 1e-6)
        w_slope /= w_sum
        w_flat /= w_sum
        w_step /= w_sum

        slope_n = np.clip(slope / max(slope_crit, 1e-6), 0.0, 1.0)
        flat_n = np.clip(flatness / max(flatness_crit, 1e-6), 0.0, 1.0)
        step_n = np.clip(step_map / max(step_crit, 1e-6), 0.0, 1.0)

        traversability = (w_slope * slope_n + w_flat * flat_n + w_step * step_n)

        # Uncertainty mask
        sigma_percentile = float(self.get_parameter('sigma_percentile').value)
        sigma_thresh = np.percentile(var, sigma_percentile)
        unsafe_unc = var > sigma_thresh

        # Cells far from observed support
        support_unknown = ~obs_mask_f

        # Separate lethal hazards from unknown terrain
        lethal_mask = (
            (step_map >= step_crit) |
            (slope >= slope_crit)
        )

        unknown_mask = support_unknown | unsafe_unc

        # OccupancyGrid values:
        # 0..95 = traversability cost, 100 = lethal obstacle, -1 = unknown
        cost = np.clip(np.rint(traversability * 95.0), 0, 95).astype(np.int16)
        cost[lethal_mask] = 100
        cost[unknown_mask] = -1

        self.get_logger().info(
            f"cells: total={cost.size} "
            f"unknown_support={int(np.sum(support_unknown))} "
            f"unknown_unc={int(np.sum(unsafe_unc))} "
            f"lethal={int(np.sum(lethal_mask))} "
            f"unknown={int(np.sum(unknown_mask))} "
            f"known={int(np.sum(cost >= 0))}"
        )

        if SCIPY_OK:
            known = cost >= 0
            tmp = cost.astype(np.float32)
            tmp[~known] = 0.0
            tmp_s = gaussian_filter(tmp, sigma=0.8)
            w = gaussian_filter(known.astype(np.float32), sigma=0.8)
            smoothed = tmp_s / np.maximum(w, 1e-6)
            cost = np.rint(smoothed).astype(np.int16)
            cost[lethal_mask] = 100
            cost[unknown_mask] = -1


        # Debug clouds
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = odom_frame

        grid_xyz = np.column_stack([grid[:, 0], grid[:, 1], mean.reshape(-1)]).astype(np.float32)
        self.pub_elev.publish(self.publish_cloud(header, grid_xyz, mean.reshape(-1).astype(np.float32)))
        self.pub_slope.publish(self.publish_cloud(header, grid_xyz, slope.reshape(-1).astype(np.float32)))
        self.pub_unc.publish(self.publish_cloud(header, grid_xyz, var.reshape(-1).astype(np.float32)))

        # OccupancyGrid
        og = OccupancyGrid()
        og.header.stamp = self.get_clock().now().to_msg()
        og.header.frame_id = odom_frame
        og.info.resolution = float(res)
        og.info.width = int(W)
        og.info.height = int(H)
        og.info.origin.position.x = float(tx - x_len / 2.0)
        og.info.origin.position.y = float(ty - y_len / 2.0)
        og.info.origin.position.z = 0.0
        og.info.origin.orientation.x = 0.0
        og.info.origin.orientation.y = 0.0
        og.info.origin.orientation.z = 0.0
        og.info.origin.orientation.w = 1.0
        og.data = cost.astype(np.int8).ravel().tolist()
        self.pub_costmap.publish(og)

        self.get_logger().info(
            f'GP update: raw={pcl_body.shape[0]} train={d_in.shape[0]} '
            f'grid={grid.shape[0]} sigma_thr={sigma_thresh:.4f} '
            f'time={wall_time() - t0:.3f}s'
        )
        self.get_logger().info(
            f'z mean[min,max]=({float(np.min(mean)):.3f},{float(np.max(mean)):.3f}) '
            f'var[min,max]=({float(np.min(var)):.5f},{float(np.max(var)):.5f}) '
            f'slope[max]={float(np.max(slope)):.3f} '
            f'step[max]={float(np.max(step_map)):.3f} '
            f'flat[max]={float(np.max(flatness)):.3f}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = GPMappingModuleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


