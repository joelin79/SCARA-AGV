#!/usr/bin/env python3
"""
Interactive 3D visualizer to validate SCARA hand-eye and coordinate transforms.
- Loads a calibration JSON (intrinsics optional, handeye preferred; extrinsics fallback)
- Lets you adjust J1/J2/J3/J4 with mouse sliders
- Draws Base (0,0,0), End Effector (EE), and Camera coordinate frames
- Draws SCARA links and extension arm for visual comparison

This tool avoids importing Arm_Control.SCARA (which opens serial).
It re-implements minimal forward kinematics consistent with the calibration pipeline:
  - EE position: x = L1*cos(J1) + L2*cos(J1+J2), y similarly, z = J3
  - EE yaw used in pipeline: yaw = J1 + J2 + J4 (degrees)
  - Base->EE rotation R_be is a yaw rotation about Z by the above yaw
  - Camera pose:
      If hand-eye present: R_bc = R_be @ R_ee2cam; t_bc = R_be @ t_ee2cam + t_be
      Else if static extrinsics present: use them directly as Base->Cam

Usage:
  python Debug/handeye_visualizer.py --calib RealSense/camera_calibration/camera_calibration.json

Mouse usage:
  - Drag 3D canvas to orbit/pan/zoom
  - Drag sliders to change J1/J2/J3/J4
  - Click "Load Calib" to choose a JSON file
"""

import argparse
import json
import math
from pathlib import Path
from typing import Optional, Tuple

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button, CheckButtons

# Optional file dialog for loading calibration at runtime
try:
    import tkinter as tk
    from tkinter import filedialog
    TK_AVAILABLE = True
except Exception:
    TK_AVAILABLE = False


# ---------- Math helpers ----------

def deg2rad(deg: float) -> float:
    return deg * math.pi / 180.0


def rotz(deg: float) -> np.ndarray:
    """Rotation matrix for yaw about Z axis (degrees)."""
    th = deg2rad(deg)
    c, s = math.cos(th), math.sin(th)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]], dtype=float)


# ---------- Visualization helpers ----------

def draw_axes(ax, R: np.ndarray, t: np.ndarray, length: float = 40.0, linewidth: float = 2.0, alpha: float = 1.0, label: Optional[str] = None):
    """Draw a coordinate frame triad at pose (R,t). X=red, Y=green, Z=blue."""
    origin = t.reshape(3)
    x_axis = origin + R @ np.array([length, 0, 0], dtype=float)
    y_axis = origin + R @ np.array([0, length, 0], dtype=float)
    z_axis = origin + R @ np.array([0, 0, length], dtype=float)

    ax.plot([origin[0], x_axis[0]], [origin[1], x_axis[1]], [origin[2], x_axis[2]], color='r', lw=linewidth, alpha=alpha)
    ax.plot([origin[0], y_axis[0]], [origin[1], y_axis[1]], [origin[2], y_axis[2]], color='g', lw=linewidth, alpha=alpha)
    ax.plot([origin[0], z_axis[0]], [origin[1], z_axis[1]], [origin[2], z_axis[2]], color='b', lw=linewidth, alpha=alpha)

    if label:
        ax.text(origin[0], origin[1], origin[2], label, color='k')


def set_axes_equal(ax):
    """Make 3D axes have equal scale."""
    x_limits = ax.get_xlim3d()
    y_limits = ax.get_ylim3d()
    z_limits = ax.get_zlim3d()

    x_range = abs(x_limits[1] - x_limits[0])
    x_middle = np.mean(x_limits)
    y_range = abs(y_limits[1] - y_limits[0])
    y_middle = np.mean(y_limits)
    z_range = abs(z_limits[1] - z_limits[0])
    z_middle = np.mean(z_limits)

    plot_radius = 0.5 * max([x_range, y_range, z_range, 1.0])
    ax.set_xlim3d([x_middle - plot_radius, x_middle + plot_radius])
    ax.set_ylim3d([y_middle - plot_radius, y_middle + plot_radius])
    ax.set_zlim3d([z_middle - plot_radius, z_middle + plot_radius])


# ---------- Visualizer core ----------

class HandEyeVisualizer:
    def __init__(self, calib_path: Optional[Path] = None):
        # Geometry
        self.L1 = 205.0
        self.L2 = 205.0
        self.extension_camera_length = 140.0  # purely for drawing the extension bar

        # Joint state (degrees, mm)
        self.j1 = 109.0
        self.j2 = -146.0
        self.j3 = 200.0
        self.j4 = 0.0

        # Calibration data
        self.R_ee2cam: Optional[np.ndarray] = None
        self.t_ee2cam: Optional[np.ndarray] = None
        self.R_base_cam_static: Optional[np.ndarray] = None
        self.t_base_cam_static: Optional[np.ndarray] = None
        # Intrinsics
        self.fx: Optional[float] = None
        self.fy: Optional[float] = None
        self.cx: Optional[float] = None
        self.cy: Optional[float] = None
        self.camera_matrix: Optional[np.ndarray] = None
        self.dist_coeffs: Optional[np.ndarray] = None
        self.width: int = 640
        self.height: int = 480

        if calib_path is not None:
            self.load_calibration(calib_path)

        # Matplotlib figure and widgets
        self.fig = plt.figure(figsize=(10, 8))
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.ax.set_title('SCARA Hand-Eye Visualizer')
        self.ax.set_xlabel('X (mm)')
        self.ax.set_ylabel('Y (mm)')
        self.ax.set_zlabel('Z (mm)')

        # Layout for sliders - adjusted to avoid overlaps
        plt.subplots_adjust(left=0.1, bottom=0.35)

        # Sliders
        axcolor = 'lightgoldenrodyellow'
        self.ax_s_j1 = plt.axes([0.1, 0.28, 0.8, 0.03], facecolor=axcolor)
        self.ax_s_j2 = plt.axes([0.1, 0.24, 0.8, 0.03], facecolor=axcolor)
        self.ax_s_j3 = plt.axes([0.1, 0.20, 0.8, 0.03], facecolor=axcolor)
        self.ax_s_j4 = plt.axes([0.1, 0.16, 0.8, 0.03], facecolor=axcolor)

        self.s_j1 = Slider(self.ax_s_j1, 'J1 (deg)', -180.0, 180.0, valinit=self.j1)
        self.s_j2 = Slider(self.ax_s_j2, 'J2 (deg)', -180.0, 180.0, valinit=self.j2)
        self.s_j3 = Slider(self.ax_s_j3, 'J3 (mm)', 0.0, 300.0, valinit=self.j3)
        self.s_j4 = Slider(self.ax_s_j4, 'J4 (deg)', -180.0, 180.0, valinit=self.j4)

        # Pixel and depth sliders
        self.ax_s_u = plt.axes([0.1, 0.12, 0.36, 0.03], facecolor=axcolor)
        self.ax_s_v = plt.axes([0.54, 0.12, 0.36, 0.03], facecolor=axcolor)
        u_init = self.cx if self.cx is not None else (self.width - 1) / 2
        v_init = self.cy if self.cy is not None else (self.height - 1) / 2
        self.s_u = Slider(self.ax_s_u, 'u (px)', 0.0, float(self.width - 1), valinit=u_init)
        self.s_v = Slider(self.ax_s_v, 'v (px)', 0.0, float(self.height - 1), valinit=v_init)
        self.ax_s_depth = plt.axes([0.1, 0.08, 0.8, 0.03], facecolor=axcolor)
        self.s_depth = Slider(self.ax_s_depth, 'Depth (mm)', 50.0, 1200.0, valinit=500.0)

        self.s_j1.on_changed(self._on_slider)
        self.s_j2.on_changed(self._on_slider)
        self.s_j3.on_changed(self._on_slider)
        self.s_j4.on_changed(self._on_slider)
        self.s_u.on_changed(self._on_slider)
        self.s_v.on_changed(self._on_slider)
        self.s_depth.on_changed(self._on_slider)

        # Buttons
        self.ax_btn_load = plt.axes([0.1, 0.02, 0.15, 0.04])
        self.btn_load = Button(self.ax_btn_load, 'Load Calib')
        self.btn_load.on_clicked(self._on_load_calib)

        self.ax_btn_reset = plt.axes([0.28, 0.02, 0.12, 0.04])
        self.btn_reset = Button(self.ax_btn_reset, 'Reset Pose')
        self.btn_reset.on_clicked(self._on_reset_pose)

        self.ax_btn_quit = plt.axes([0.82, 0.02, 0.08, 0.04])
        self.btn_quit = Button(self.ax_btn_quit, 'Quit')
        self.btn_quit.on_clicked(lambda evt: plt.close(self.fig))

        # Checkboxes for options
        self.ax_checks = plt.axes([0.72, 0.30, 0.18, 0.07], facecolor=axcolor)
        self.use_undistort = True
        self.show_simplified = True
        self.checks = CheckButtons(self.ax_checks, ['Undistort', 'Show simplified'], [self.use_undistort, self.show_simplified])
        self.checks.on_clicked(self._on_checks)

        self._redraw()

    # ----- Calibration -----
    def load_calibration(self, path: Path):
        try:
            with open(path, 'r') as f:
                data = json.load(f)
        except Exception as e:
            print(f"Failed to load calibration file: {e}")
            return

        # intrinsics (optional but recommended)
        intr = data.get('intrinsics', None)
        if intr is not None:
            try:
                self.fx = float(intr.get('fx', self.fx if self.fx is not None else 606.0))
                self.fy = float(intr.get('fy', self.fy if self.fy is not None else 606.0))
                self.cx = float(intr.get('cx', self.cx if self.cx is not None else (intr.get('width', 640) - 1) / 2))
                self.cy = float(intr.get('cy', self.cy if self.cy is not None else (intr.get('height', 480) - 1) / 2))
                self.width = int(intr.get('width', self.width))
                self.height = int(intr.get('height', self.height))
            except Exception as e:
                print(f"Invalid intrinsics block: {e}")

        # camera_matrix and distortion (optional)
        if 'camera_matrix' in data:
            try:
                self.camera_matrix = np.array(data['camera_matrix'], dtype=float)
            except Exception as e:
                print(f"Invalid camera_matrix: {e}")
        if 'dist_coeffs' in data:
            try:
                self.dist_coeffs = np.array(data['dist_coeffs'], dtype=float)
            except Exception as e:
                print(f"Invalid dist_coeffs: {e}")

        # handeye preferred
        he = data.get('handeye', None)
        if he is not None:
            try:
                R = np.array(he['R_ee2cam'], dtype=float)
                t = np.array(he['t_ee2cam'], dtype=float).reshape(3)
                if R.shape == (3, 3) and t.shape == (3,):
                    self.R_ee2cam = R
                    self.t_ee2cam = t

                    Rz180 = np.array([[-1, 0, 0],
                          [ 0,-1, 0],
                          [ 0, 0, 1]], float)      # flip to opposite XY side
                    Rx180 = np.array([[ 1, 0, 0],
                          [ 0,-1, 0],
                          [ 0, 0,-1]], float)      # flip cam Z to point down

                    Rcorr = Rx180 @ Rz180                      # apply both flips
                    self.R_ee2cam = Rcorr @ self.R_ee2cam
                    self.t_ee2cam = Rcorr @ self.t_ee2cam

                    Rz90 = np.array([[0., 1., 0.],
                    [-1.,  0., 0.],
                    [0.,  0., 1.]], float)  # CW 90° around +Z (right-handed)

                    # Rotate camera axes about its own origin: post-multiply
                    self.R_ee2cam = self.R_ee2cam @ Rz90

                    print('Loaded hand-eye (EE->Cam) from calibration file')
            except Exception as e:
                print(f"Invalid handeye block: {e}")

        # legacy extrinsics fallback (static camera in base frame)
        ex = data.get('extrinsics', None)
        if ex is not None:
            try:
                R = np.array(ex['rotation_matrix'], dtype=float)
                t = np.array(ex['translation_vector'], dtype=float).reshape(3)
                if R.shape == (3, 3) and t.shape == (3,):
                    self.R_base_cam_static = R
                    self.t_base_cam_static = t
                    print('Loaded static extrinsics (Base->Cam) from calibration file')
            except Exception as e:
                print(f"Invalid extrinsics block: {e}")

        # Update pixel slider ranges to match new image dimensions
        self._update_pixel_sliders()

    def _update_pixel_sliders(self):
        """Update pixel slider ranges and values based on current image dimensions"""
        try:
            # Update slider ranges
            self.s_u.valmax = float(self.width - 1)
            self.s_v.valmax = float(self.height - 1)
            
            # Update current values if they're out of range
            u_val = self.s_u.val
            v_val = self.s_v.val
            
            if u_val > self.width - 1:
                u_val = self.cx if self.cx is not None else (self.width - 1) / 2
            if v_val > self.height - 1:
                v_val = self.cy if self.cy is not None else (self.height - 1) / 2
                
            self.s_u.set_val(u_val)
            self.s_v.set_val(v_val)
            
            # Update slider labels
            self.s_u.label.set_text(f'u (px) [0-{self.width-1}]')
            self.s_v.label.set_text(f'v (px) [0-{self.height-1}]')
            
        except Exception as e:
            print(f"Error updating pixel sliders: {e}")

    # ----- Kinematics and poses -----
    def _fk_xy(self, j1_deg: float, j2_deg: float) -> Tuple[float, float]:
        j1 = deg2rad(j1_deg)
        j2 = deg2rad(j2_deg)
        x = self.L1 * math.cos(j1) + self.L2 * math.cos(j1 + j2)
        y = self.L1 * math.sin(j1) + self.L2 * math.sin(j1 + j2)
        return x, y

    def current_ee_pose(self) -> Tuple[np.ndarray, np.ndarray]:
        """Return (R_be, t_be). R_be uses yaw = J1+J2+J4 to match pipeline."""
        x, y = self._fk_xy(self.j1, self.j2)
        t_be = np.array([x, y, self.j3], dtype=float)
        yaw = self.j1 + self.j2 + self.j4
        R_be = rotz(yaw)
        return R_be, t_be

    def current_cam_pose(self) -> Optional[Tuple[np.ndarray, np.ndarray]]:
        R_be, t_be = self.current_ee_pose()

        if self.R_ee2cam is not None and self.t_ee2cam is not None:
            R_bc = R_be @ self.R_ee2cam
            t_bc = R_be @ self.t_ee2cam.reshape(3) + t_be
            return R_bc, t_bc
        elif self.R_base_cam_static is not None and self.t_base_cam_static is not None:
            return self.R_base_cam_static, self.t_base_cam_static
        else:
            return None

    # ----- UI callbacks -----
    def _on_slider(self, val):
        self.j1 = self.s_j1.val
        self.j2 = self.s_j2.val
        self.j3 = self.s_j3.val
        self.j4 = self.s_j4.val
        self._redraw()

    def _on_reset_pose(self, evt):
        self.s_j1.reset()
        self.s_j2.reset()
        self.s_j3.reset()
        self.s_j4.reset()

    def _on_load_calib(self, evt):
        path: Optional[str] = None
        if TK_AVAILABLE:
            try:
                root = tk.Tk()
                root.withdraw()
                path = filedialog.askopenfilename(
                    title='Select calibration JSON',
                    filetypes=[('JSON files', '*.json'), ('All files', '*.*')]
                )
                root.destroy()
            except Exception as e:
                print(f"File dialog error: {e}")
        if not path:
            print('No file selected')
            return
        self.load_calibration(Path(path))
        self._redraw()

    def _on_checks(self, label):
        if label == 'Undistort':
            self.use_undistort = not self.use_undistort
        elif label == 'Show simplified':
            self.show_simplified = not self.show_simplified
        self._redraw()

    # ----- Drawing -----
    def _redraw(self):
        self.ax.cla()
        self.ax.set_title('SCARA Hand-Eye Visualizer')
        self.ax.set_xlabel('X (mm)')
        self.ax.set_ylabel('Y (mm)')
        self.ax.set_zlabel('Z (mm)')

        # Draw base axes at origin
        draw_axes(self.ax, np.eye(3), np.zeros(3), length=50.0, linewidth=2.5, alpha=0.9, label='Base')

        # Draw arm links in XY plane at height J3
        j1_rad = deg2rad(self.j1)
        j2_rad = deg2rad(self.j2)
        shoulder = np.array([0.0, 0.0, self.j3])
        elbow = np.array([self.L1 * math.cos(j1_rad), self.L1 * math.sin(j1_rad), self.j3])
        ee_xy = np.array([self.L1 * math.cos(j1_rad) + self.L2 * math.cos(j1_rad + j2_rad),
                          self.L1 * math.sin(j1_rad) + self.L2 * math.sin(j1_rad + j2_rad),
                          self.j3])

        self.ax.plot([shoulder[0], elbow[0]], [shoulder[1], elbow[1]], [shoulder[2], elbow[2]], 'k-', lw=3.0, alpha=0.8)
        self.ax.plot([elbow[0], ee_xy[0]], [elbow[1], ee_xy[1]], [elbow[2], ee_xy[2]], 'k-', lw=3.0, alpha=0.8)

        # EE frame (uses yaw J1+J2+J4 per pipeline)
        R_be, t_be = self.current_ee_pose()
        draw_axes(self.ax, R_be, t_be, length=35.0, linewidth=2.0, alpha=0.9, label='EE')

        # Extension arm (for visual reference only): along EE yaw by extension length
        yaw = self.j1 + self.j2 + self.j4
        yaw_rad = deg2rad(yaw)
        ext_end = t_be + np.array([self.extension_camera_length * math.cos(yaw_rad),
                                   self.extension_camera_length * math.sin(yaw_rad),
                                   0.0])
        self.ax.plot([t_be[0], ext_end[0]], [t_be[1], ext_end[1]], [t_be[2], ext_end[2]], color='#8888ff', lw=2.0, alpha=0.7)
        self.ax.scatter([ext_end[0]], [ext_end[1]], [ext_end[2]], color='#6666ff', s=30, alpha=0.9, label='Extension tip (~140mm)')

        # Camera frame
        cam_pose = self.current_cam_pose()
        if cam_pose is not None:
            R_bc, t_bc = cam_pose
            draw_axes(self.ax, R_bc, t_bc, length=35.0, linewidth=2.0, alpha=0.9, label='Cam')
            # Connector line from EE to Cam for visualizing offset
            self.ax.plot([t_be[0], t_bc[0]], [t_be[1], t_bc[1]], [t_be[2], t_bc[2]], linestyle='--', color='#ff8888', alpha=0.6)

            # Compute and draw pixel ray and transformed point(s)
            u = float(self.s_u.val)
            v = float(self.s_v.val)
            Z = float(self.s_depth.val)
            # Camera-space direction and point at depth Z
            if self.use_undistort and self.camera_matrix is not None and self.dist_coeffs is not None:
                pts = np.array([[[u, v]]], dtype=np.float32)
                try:
                    und = np.squeeze(cv2.undistortPoints(pts, self.camera_matrix, self.dist_coeffs, P=None))
                    x_norm = float(und[0]); y_norm = float(und[1])
                except Exception:
                    # fallback pinhole
                    x_norm = (u - (self.cx if self.cx is not None else (self.width - 1) / 2.0)) / (self.fx if self.fx is not None else 606.0)
                    y_norm = (v - (self.cy if self.cy is not None else (self.height - 1) / 2.0)) / (self.fy if self.fy is not None else 606.0)
            else:
                x_norm = (u - (self.cx if self.cx is not None else (self.width - 1) / 2.0)) / (self.fx if self.fx is not None else 606.0)
                y_norm = (v - (self.cy if self.cy is not None else (self.height - 1) / 2.0)) / (self.fy if self.fy is not None else 606.0)

            cam_dir = np.array([x_norm, y_norm, 1.0], dtype=float)
            cam_dir = cam_dir / np.linalg.norm(cam_dir)
            cam_point = cam_dir * Z

            # Draw the ray from camera origin to a point beyond the chosen depth
            p0 = t_bc
            pZ = t_bc + R_bc @ cam_point
            p_far = t_bc + R_bc @ (cam_dir * (Z * 1.5))
            self.ax.plot([p0[0], p_far[0]], [p0[1], p_far[1]], [p0[2], p_far[2]], color='#00aaee', lw=1.5, alpha=0.9)
            self.ax.scatter([pZ[0]], [pZ[1]], [pZ[2]], color='#0077cc', s=30, label='Cam-space point @Z')

            # Calibrated/base point using hand-eye or static extrinsics:
            # In our coordinate convention, pZ is already base-frame because we applied R_bc,t_bc
            # So we simply report it as calibrated point
            calibrated_point = pZ
            self.ax.scatter([calibrated_point[0]], [calibrated_point[1]], [calibrated_point[2]], color='#00cc44', s=45, label='Arm point (calibrated)')

            # Simplified mapping (ObjectDetection fallback)
            if self.show_simplified:
                yaw_deg = self.j1 + self.j2 + self.j4
                yaw_rad = deg2rad(yaw_deg)
                fx = self.fx if self.fx is not None else 615.0
                fy = self.fy if self.fy is not None else 615.0
                cx = self.cx if self.cx is not None else 320.0
                cy = self.cy if self.cy is not None else 240.0
                cam_x = (u - cx) * Z / fx
                cam_y = (v - cy) * Z / fy
                cam_z = Z
                cos_yaw = math.cos(yaw_rad)
                sin_yaw = math.sin(yaw_rad)
                world_dx = cam_x * cos_yaw - cam_y * sin_yaw
                world_dy = cam_x * sin_yaw + cam_y * cos_yaw
                arm_x = t_bc[0] + world_dx
                arm_y = t_bc[1] + world_dy
                arm_z = t_bc[2] - cam_z
                simplified_point = np.array([arm_x, arm_y, arm_z], dtype=float)
                self.ax.scatter([simplified_point[0]], [simplified_point[1]], [simplified_point[2]], color='#ff9900', s=45, label='Arm point (simplified)')

        # Cosmetic: base reach circle at Z=0
        theta = np.linspace(0, 2*np.pi, 200)
        max_reach = self.L1 + self.L2 + self.extension_camera_length
        circle_x = max_reach * np.cos(theta)
        circle_y = max_reach * np.sin(theta)
        circle_z = np.zeros_like(circle_x)
        self.ax.plot(circle_x, circle_y, circle_z, 'k--', alpha=0.2)

        # Set view and limits
        self.ax.view_init(elev=25, azim=-60)
        self.ax.set_xlim(-500, 500)
        self.ax.set_ylim(-500, 500)
        self.ax.set_zlim(0, 400)
        set_axes_equal(self.ax)

        # Legend-like texts
        info = []
        info.append(f"J1={self.j1:.1f}°  J2={self.j2:.1f}°  J3={self.j3:.1f}  J4={self.j4:.1f}°")
        if self.R_ee2cam is not None:
            info.append("handeye: yes")
        if self.R_base_cam_static is not None:
            info.append("extrinsics: yes")
        info.append(f"u={float(self.s_u.val):.1f}, v={float(self.s_v.val):.1f}, Z={float(self.s_depth.val):.0f}mm")
        info.append(f"undistort={'on' if self.use_undistort else 'off'} | simplified={'on' if self.show_simplified else 'off'}")
        self.ax.text2D(0.02, 0.98, " | ".join(info), transform=self.ax.transAxes)
        self.ax.legend(loc='upper right')

        plt.draw()

    # ----- Main loop -----
    def show(self):
        plt.show()


def main():
    parser = argparse.ArgumentParser(description='Interactive 3D hand-eye visualizer')
    parser.add_argument('--calib', type=str, default='', help='Path to calibration JSON file')
    args = parser.parse_args()

    calib_path = Path(args.calib) if args.calib else None
    vis = HandEyeVisualizer(calib_path=calib_path)
    vis.show()


if __name__ == '__main__':
    main() 