# Spinning 3D Donut Renderer

A terminal-based 3D rotating torus ("donut") renderer in Python, featuring:

- **Parametric Torus Geometry**: Minor (tube) and major (ring) radii define the shape.
- **3D Rotation**: Continuous rotation about X and Z axes for a dynamic effect.
- **Perspective Projection**: Points are projected onto the terminal screen with depth scaling.
- **Depth Buffering**: Z-buffer ensures correct visibility of overlapping surfaces.
- **Lambertian Shading**: Grayscale shading based on surface normals and a fixed light source.
- **ANSI Escape Codes**: Uses 24‑bit ANSI color to render smooth grayscale blocks.

## Demo

![Spinning Donut Demo](assets/donut_demo.gif)

## Features

- **Automatic Scaling**: Fits the donut to any terminal size with configurable margin.
- **Adjustable Geometry**: Change tube and ring radii directly in the script.
- **Lightweight**: Pure Python, no external dependencies beyond the standard library.
- **Cross Platform**: Works in Linux, macOS, and Windows terminals that support ANSI escape codes.

## Installation

1. **Clone the repository**:

   ```bash
   git clone https://github.com/ezAldinWaez/terminal-donut.git
   cd terminal-donut
   ```

2. **Ensure Python 3.6+ is installed**.

No additional packages are required.

## Usage

Run the script directly:

```bash
python donut.py
```

### Configuration

Inside `donut.py`, you can adjust the following parameters:

- `tube_radius` (float): Minor radius.
- `ring_radius` (float): Major radius.
- `viewer_distance` (float): Distance from the camera to the torus center.
- `margin` (float, `0 < margin <= 1`): Fraction of terminal to occupy.
- `frame_delay` (float): Seconds to wait between frames.

Example instantiation:

```python
renderer = TorusRenderer(
    tube_radius=0.8,
    ring_radius=2.5,
    viewer_distance=8.0,
    margin=0.8
)
renderer.render(frame_delay=0.02)
```

## License

This project is licensed under the MIT License.
