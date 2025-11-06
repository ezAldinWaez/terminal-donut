"""
Terminal-based 3D rotating torus ("donut") renderer.

This module implements a parametric torus surface, applies 3D rotations, performs perspective
projection, and renders the result in the terminal using ANSI escape codes for grayscale shading.

The shading employs a Lambertian reflectance model with a fixed light source.

Mathematical Overview:
    - Torus parametric equations:
        - `x_local = (R2 + R1 * cos(θ)) * cos(φ)`
        - `y_local = (R2 + R1 * cos(θ)) * sin(φ)`
        - `z_local = R1 * sin(θ)`
    - 3D rotations: first about X-axis by angle A, then about Z-axis by angle B.
    - Perspective projection: (x, y, z) -> (col, row) with depth-based scaling.
    - Depth buffering: z-buffer to resolve visibility.
    - Lambertian shading: color intensity ∝ max(0, n · L).
"""

from math import sin, cos, sqrt, pi as π
import shutil
import sys
import time
import types

# ANSI escape sequences for terminal control and styling.
_CLEAR_SCREEN = "\033[2J"
_HIDE_CURSOR = "\033[?25l"
_SHOW_CURSOR = "\033[?25h"
_CURSOR_HOME = "\033[H"
_RESET_STYLE = "\033[0m"


class TerminalController:
    """
    Context manager to configure terminal state for animation.

    On enter, clears the screen and hides the cursor. On exit, shows the cursor and clears the
    screen to restore the terminal.

    Usage:
        ```python
        with TerminalController():
            # animation code
        ```
    """

    def __enter__(self) -> "TerminalController":
        """
        Prepare the terminal for rendering.

        Returns:
            TerminalController: The controller instance for use in a with-statement.
        """
        sys.stdout.write(_CLEAR_SCREEN)
        sys.stdout.write(_HIDE_CURSOR)
        sys.stdout.flush()
        return self

    def __exit__(
        self,
        exc_type: type[BaseException],
        exc_val: BaseException,
        exc_tb: types.TracebackType,
    ) -> None:
        """
        Restore terminal state after rendering.

        Args:
            exc_type: Exception type if raised inside the context.
            exc_val: Exception value if raised inside the context.
            exc_tb: Traceback if exception was raised.
        """
        sys.stdout.write(_SHOW_CURSOR)
        sys.stdout.write(_CLEAR_SCREEN)
        sys.stdout.flush()


class TorusRenderer:
    """
    Renders a spinning torus (donut) in the terminal using grayscale shading.

    A torus is defined by two radii: tube_radius (minor) and ring_radius (major).
    Rendering involves:

        1. Parametric sampling over angles theta (tube) and phi (ring).
        2. 3D rotations about the X and Z axes.
        3. Perspective projection to screen coordinates.
        4. Depth buffering to resolve visibility.
        5. Lambertian shading with a fixed light vector.

    Attributes:
        tube_radius (float): Minor radius (R1) of the torus tube.
        ring_radius (float): Major radius (R2) of the torus ring.
        viewer_distance (float): Distance (K2) from viewer to torus center.
        margin (float): Fraction of terminal dimension to occupy (0 < margin <= 1).
        A (float): Current rotation angle about the X-axis (radians).
        B (float): Current rotation angle about the Z-axis (radians).
        K1 (float): Projection scaling factor, computed per frame.
    """

    def __init__(
        self,
        tube_radius: float = 1.0,
        ring_radius: float = 2.0,
        viewer_distance: float = 5.0,
        margin: float = 0.9,
    ) -> None:
        """
        Initialize the torus renderer with given geometry and scaling.

        Args:
            tube_radius (float): Minor radius R1 of the torus tube.
            ring_radius (float): Major radius R2 of the torus ring.
            viewer_distance (float): Distance K2 from viewer to torus center.
            margin (float): Fraction of terminal size to use (e.g., 0.9 for 90%).
        """
        self.tube_radius = tube_radius
        self.ring_radius = ring_radius
        self.viewer_distance = viewer_distance
        self.margin = margin

        # Rotation angles around X and Z axes
        self.A = 0.0
        self.B = 0.0

        # Projection scale, set each frame after querying terminal size
        self.K1: float

    def _compute_lighting(self, nx: float, ny: float, nz: float) -> int:
        """
        Compute grayscale intensity using Lambertian reflectance.

        The light vector is fixed at `L = (0, 1, -1)` and normalized.

        `Intensity = base + scale * max(0, n · L)`.

        Args:
            nx (float): X component of surface normal (unit vector).
            ny (float): Y component of surface normal.
            nz (float): Z component of surface normal.

        Returns:
            int: Grayscale level in [64, 255], mapped from the dot product.
        """
        # Unnormalized light direction
        lx, ly, lz = 0.0, 1.0, -1.0
        norm = sqrt(lx * lx + ly * ly + lz * lz)
        lx, ly, lz = lx / norm, ly / norm, lz / norm

        # Lambertian dot product
        dot = nx * lx + ny * ly + nz * lz
        brightness = max(dot, 0.0)

        # Map [0,1] to [64,255] for ANSI gray scale
        return int(64 + brightness * (255 - 64))

    def _project(
        self, x: float, y: float, z: float
    ) -> tuple[int, int, float]:
        """
        Project a 3D point into 2D terminal coordinates with depth.

        Uses simple perspective: `screen_x = K1 * x / (z + K2)`, etc.

        Args:
            x (float): X coordinate after rotation.
            y (float): Y coordinate after rotation.
            z (float): Z coordinate after rotation.

        Returns:
            tuple:
                - int: Column index in terminal.
                - int: Row index in terminal.
                - float: Inverse depth (`1/(z + viewer_distance)`).
        """
        ooz = 1.0 / (z + self.viewer_distance)
        col = int(self.K1 * x * ooz + self.width / 2)
        row = int(self.K1 * y * ooz + self.height / 2)
        return col, row, ooz

    def _compute_frame(self) -> None:
        """
        Compute and render a single frame of the rotating torus.

        This involves:
            1. Querying terminal size.
            2. Computing projection scale K1 based on margin.
            3. Initializing z-buffer and output character buffer.
            4. Sampling the torus surface by angles theta, phi.
            5. Applying rotations, projection, depth test, and shading.
            6. Writing the frame to stdout.
        """
        # 1. Terminal dimensions
        self.width, self.height = shutil.get_terminal_size()

        # 2. Compute K1 so torus fits within 'margin' of the smaller dimension
        dim = min(self.width, self.height) * self.margin
        self.K1 = dim * self.viewer_distance * 3.0 / \
            (8.0 * (self.tube_radius + self.ring_radius))

        # 3. Buffers for depth and output
        size = self.width * self.height
        zbuffer = [0.0] * size
        output = [' '] * size

        # Precompute rotation sines/cosines
        cosA, sinA = cos(self.A), sin(self.A)
        cosB, sinB = cos(self.B), sin(self.B)

        # Sampling steps
        Δϑ = 0.07
        Δφ = 0.02

        # 4. Loop over torus angles
        ϑ = 0.0
        while ϑ < 2*π:
            cosϑ, sinϑ = cos(ϑ), sin(ϑ)
            φ = 0.0
            while φ < 2*π:
                cosφ, sinφ = cos(φ), sin(φ)

                # Parametric torus point before rotation
                tube_x = self.ring_radius + self.tube_radius * cosϑ
                tube_y = self.tube_radius * sinϑ

                # 5. Apply rotations
                x = tube_x * (cosB * cosφ + sinA * sinB * sinφ) - tube_y * cosA * sinB
                y = tube_x * (sinB * cosφ - sinA * cosB * sinφ) + tube_y * cosA * cosB
                z = tube_x * cosA * sinφ + tube_y * sinA

                # Compute surface normal components
                nx = cosφ * cosϑ * cosB - cosϑ * sinφ * sinA * sinB - sinϑ * cosA * sinB
                ny = cosφ * cosϑ * sinB + cosϑ * sinφ * sinA * cosB + sinϑ * cosA * cosB
                nz = cosϑ * sinφ * cosA - sinϑ * sinA

                # Project and depth test
                col, row, ooz = self._project(x, y, z)
                idx = row * self.width + col
                if 0 <= row < self.height and 0 <= col < self.width and ooz > zbuffer[idx]:
                    zbuffer[idx] = ooz
                    gray = self._compute_lighting(nx, ny, nz)
                    output[idx] = f"\033[38;2;{gray};{gray};{gray}m█{_RESET_STYLE}"
                φ += Δφ
            ϑ += Δϑ

        # 6. Render to terminal
        sys.stdout.write(_CURSOR_HOME)
        for r in range(self.height):
            start = r * self.width
            end = start + self.width
            sys.stdout.write(''.join(output[start:end]) + '\n')
        sys.stdout.flush()

    def render(self, frame_delay: float = 0.01) -> None:
        """
        Continuously render frames until interrupted by the user.

        Args:
            frame_delay: Time to sleep between frames (in seconds).
        """
        # Rotation increments per frame (radians)
        ΔA = 0.07
        ΔB = 0.03

        try:
            while True:
                self._compute_frame()
                self.A += ΔA
                self.B += ΔB
                time.sleep(frame_delay)
        except KeyboardInterrupt:
            # Graceful exit on Ctrl-C
            pass


if __name__ == '__main__':
    with TerminalController():
        renderer = TorusRenderer(
            tube_radius=1.0,
            ring_radius=3.0,
            viewer_distance=15.0,
            margin=1.0
        )
        renderer.render(frame_delay=0.0)
