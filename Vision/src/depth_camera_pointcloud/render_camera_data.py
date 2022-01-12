
import math
import numpy as np
import cv2
import pyrealsense2 as rs

# Code used was provided as an example for the librealsense package and then turned into a module; can be found here:
# https://github.com/IntelRealSense/librealsense/blob/development/wrappers/python/examples/opencv_pointcloud_viewer.py
class AppState:
    def __init__(self, *args, **kwargs):
        self.WIN_NAME = 'RealSense'
        self.pitch, self.yaw = math.radians(-10), math.radians(-15)
        self.translation = np.array([0, 0, -1], dtype=np.float32)
        self.distance = 2
        self.prev_mouse = 0, 0
        self.mouse_btns = [False, False, False]
        self.paused = False
        self.decimate = 1
        self.scale = True
        self.color = True

    def reset(self):
        self.pitch, self.yaw, self.distance = 0, 0, 2
        self.translation[:] = 0, 0, -1

    @property
    def rotation(self):
        Rx, _ = cv2.Rodrigues((self.pitch, 0, 0))
        Ry, _ = cv2.Rodrigues((0, self.yaw, 0))
        return np.dot(Ry, Rx).astype(np.float32)

    @property
    def pivot(self):
        return self.translation + np.array((0, 0, self.distance), dtype=np.float32)

class RenderCameraData:
    def __init__(self, state, out):
        self.state = state
        self.out = out

    def mouse_cb(self, event, x, y, flags, param):

        if event == cv2.EVENT_LBUTTONDOWN:
            self.state.mouse_btns[0] = True

        if event == cv2.EVENT_LBUTTONUP:
            self.state.mouse_btns[0] = False

        if event == cv2.EVENT_RBUTTONDOWN:
            self.state.mouse_btns[1] = True

        if event == cv2.EVENT_RBUTTONUP:
            self.state.mouse_btns[1] = False

        if event == cv2.EVENT_MBUTTONDOWN:
            self.state.mouse_btns[2] = True

        if event == cv2.EVENT_MBUTTONUP:
            self.state.mouse_btns[2] = False

        if event == cv2.EVENT_MOUSEMOVE:

            h, w = self.out.shape[:2]
            dx, dy = x - self.state.prev_mouse[0], y - self.state.prev_mouse[1]

            if self.state.mouse_btns[0]:
                self.state.yaw += float(dx) / w * 2
                self.state.pitch -= float(dy) / h * 2

            elif self.state.mouse_btns[1]:
                dp = np.array((dx / w, dy / h, 0), dtype=np.float32)
                self.state.translation -= np.dot(self.state.rotation, dp)

            elif self.state.mouse_btns[2]:
                dz = math.sqrt(dx**2 + dy**2) * math.copysign(0.01, -dy)
                self.state.translation[2] += dz
                self.state.distance -= dz

        if event == cv2.EVENT_MOUSEWHEEL:
            dz = math.copysign(0.1, flags)
            self.state.translation[2] += dz
            self.state.distance -= dz

        self.state.prev_mouse = (x, y)

    def project(self, v):
        """project 3d vector array to 2d"""
        h, w = self.out.shape[:2]
        view_aspect = float(h)/w

        # ignore divide by zero for invalid depth
        with np.errstate(divide='ignore', invalid='ignore'):
            proj = v[:, :-1] / v[:, -1, np.newaxis] * \
                (w*view_aspect, h) + (w/2.0, h/2.0)

        # near clipping
        znear = 0.03
        proj[v[:, 2] < znear] = np.nan
        return proj


    def view(self, v):
        """apply view transformation on vector array"""
        return np.dot(v - self.state.pivot, self.state.rotation) + self.state.pivot - self.state.translation


    def line3d(self, out, pt1, pt2, color=(0x80, 0x80, 0x80), thickness=1):
        """draw a 3d line from pt1 to pt2"""

        p0 = self.project(pt1.reshape(-1, 3))[0]
        p1 = self.project(pt2.reshape(-1, 3))[0]
        if np.isnan(p0).any() or np.isnan(p1).any():
            return
        p0 = tuple(p0.astype(int))
        p1 = tuple(p1.astype(int))
        rect = (0, 0, out.shape[1], out.shape[0])
        inside, p0, p1 = cv2.clipLine(rect, p0, p1)
        if inside:
            cv2.line(out, p0, p1, color, thickness, cv2.LINE_AA)


    def grid(self, out, pos, rotation=np.eye(3), size=1, n=10, color=(0x80, 0x80, 0x80)):
        """draw a grid on xz plane"""
        pos = np.array(pos)
        s = size / float(n)
        s2 = 0.5 * size
        for i in range(0, n+1):
            x = -s2 + i*s
            self.line3d(out, self.view(pos + np.dot((x, 0, -s2), rotation)),
                self.view(pos + np.dot((x, 0, s2), rotation)), color)
        for i in range(0, n+1):
            z = -s2 + i*s
            self.line3d(out, self.view(pos + np.dot((-s2, 0, z), rotation)),
                self.view(pos + np.dot((s2, 0, z), rotation)), color)


    def axes(self, out, pos, rotation=np.eye(3), size=0.075, thickness=2):
        """draw 3d axes"""
        self.line3d(out, pos, pos +
            np.dot((0, 0, size), rotation), (0xff, 0, 0), thickness)
        self.line3d(out, pos, pos +
            np.dot((0, size, 0), rotation), (0, 0xff, 0), thickness)
        self.line3d(out, pos, pos +
            np.dot((size, 0, 0), rotation), (0, 0, 0xff), thickness)


    def frustum(self, out, intrinsics, color=(0x40, 0x40, 0x40)):
        """draw camera's frustum"""
        orig = self.view([0, 0, 0])
        w, h = intrinsics.width, intrinsics.height

        for d in range(1, 6, 2):
            def get_point(x, y):
                p = rs.rs2_deproject_pixel_to_point(intrinsics, [x, y], d)
                self.line3d(out, orig, self.view(p), color)
                return p

            top_left = get_point(0, 0)
            top_right = get_point(w, 0)
            bottom_right = get_point(w, h)
            bottom_left = get_point(0, h)

            self.line3d(out, self.view(top_left), self.view(top_right), color)
            self.line3d(out, self.view(top_right), self.view(bottom_right), color)
            self.line3d(out, self.view(bottom_right), self.view(bottom_left), color)
            self.line3d(out, self.view(bottom_left), self.view(top_left), color)


    def pointcloud(self, out, verts, texcoords, color, painter=True):
        """draw point cloud with optional painter's algorithm"""
        if painter:
            # Painter's algo, sort points from back to front

            # get reverse sorted indices by z (in view-space)
            # https://gist.github.com/stevenvo/e3dad127598842459b68
            v = self.view(verts)
            s = v[:, 2].argsort()[::-1]
            proj = self.project(v[s])
        else:
            proj = self.project(self.view(verts))

        if self.state.scale:
            proj *= 0.5**self.state.decimate

        h, w = out.shape[:2]

        # proj now contains 2d image coordinates
        j, i = proj.astype(np.uint32).T

        # create a mask to ignore out-of-bound indices
        im = (i >= 0) & (i < h)
        jm = (j >= 0) & (j < w)
        m = im & jm

        cw, ch = color.shape[:2][::-1]
        if painter:
            # sort texcoord with same indices as above
            # texcoords are [0..1] and relative to top-left pixel corner,
            # multiply by size and add 0.5 to center
            v, u = (texcoords[s] * (cw, ch) + 0.5).astype(np.uint32).T
        else:
            v, u = (texcoords * (cw, ch) + 0.5).astype(np.uint32).T
        # clip texcoords to image
        np.clip(u, 0, ch-1, out=u)
        np.clip(v, 0, cw-1, out=v)

        # perform uv-mapping
        out[i[m], j[m]] = color[u[m], v[m]]

