# Gazebo Web Viewer (cloud)

This service streams the **real Gazebo Harmonic GUI** to a browser using Xvfb,
x11vnc and noVNC. It is not a prerecorded video or a rendered mock-up.
The noVNC client preserves Gazebo's normal mouse controls: left-drag to orbit,
middle-drag to pan, and wheel to zoom.

## Deploy on Railway

1. Create a new Railway project from this repository.
2. Set the service root to the repository root and builder to Dockerfile.
3. Set the Dockerfile path to `deploy/gazebo-web-viewer/Dockerfile`.
4. Add a secret environment variable `VNC_PASSWORD` with a unique random
   value of at least 12 characters. Never commit this secret.
5. Expose target port `6080`, then generate a public HTTPS domain.
6. Open `https://<your-domain>/vnc.html?autoconnect=true&resize=remote`.
   Enter the VNC password when prompted.

Optional variables:
- `SCREEN_RESOLUTION` (default `1600x900x24`)
- `PW_AUTO_DRINK` (default `1`; set `0` to disable autonomous drink requests)
- `PW_DRINK_FIRST` (default `45` seconds)
- `PW_DRINK_GAP_MIN` / `PW_DRINK_GAP_MAX` (defaults `120` / `180` seconds)

## Notes

- The image builds the Gazebo GraspCarry plugin and the ROS 2 workspace.
- At startup it launches the real `work_area.sdf`, ROS-Gazebo bridge, follower,
  Nav2 and DrinkTask nodes using the repository's existing startup scripts.
- The person simulation automatically requests a drink by default, so the
  complete behavior can be observed without a terminal.
- Software rendering is used because managed cloud containers generally do
  not expose a GPU. A service with at least 4 vCPU and 8 GB RAM is recommended;
  8 vCPU and 16 GB RAM is preferable for smoother Gazebo GUI rendering.
- The public endpoint is protected by VNC password authentication. Use a
  long, unique secret and rotate it if it is exposed.
- This is an always-on interactive simulator service, not a GitHub Actions job.
  Hosting and compute charges depend on the selected provider and runtime.
