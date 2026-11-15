# Gazebo Web Viewer (same simulation host)

This exposes the **existing Gazebo desktop** to a browser on the same machine
or a trusted local network. It does not create another simulator, move the
simulation to a cloud provider, or use a public relay. The browser receives
the actual Gazebo GUI through x11vnc + noVNC; mouse controls remain Gazebo's
own controls (left-drag orbit, middle-drag pan, wheel zoom).

## Install on the machine that runs Gazebo

On Ubuntu:

```bash
sudo apt-get update
sudo apt-get install -y x11vnc novnc websockify
```

The host must already have its normal graphical desktop and Gazebo GUI running
on the active X display (the VLA scripts default to `:0`).

## Start / stop

From the repository root:

```bash
export DISPLAY=:0
export VNC_PASSWORD='use-a-unique-password-at-least-12-characters'
bash tools/start_gazebo_web_viewer.sh
```

Open the printed local URL in a browser and enter the VNC password.

Stop only the web viewer (Gazebo keeps running):

```bash
bash tools/stop_gazebo_web_viewer.sh
```

## Access from another device on the same LAN

By default, the service binds to `127.0.0.1` and is not exposed to the LAN.
To access it from another computer on the same trusted LAN:

```bash
LISTEN_ADDRESS=0.0.0.0 VNC_PASSWORD='your-unique-long-password' \
  bash tools/start_gazebo_web_viewer.sh
```

Then open `http://<simulation-host-LAN-IP>:6080/vnc.html?autoconnect=true`.
Allow TCP port 6080 only on the trusted LAN firewall. Do not port-forward it
to the public internet. The VNC password protects the desktop session, but
plain HTTP/noVNC is intended for a trusted LAN, not an untrusted network.

## Existing VLA camera tools

Once connected, use the existing Gazebo GUI and its native mouse controls.
The project already provides camera presets and follow mode:

```bash
bash tools/open_gui.sh free
bash tools/open_gui.sh wide
bash tools/open_gui.sh top
python3 tools/gui_follow.py robot
python3 tools/gui_follow.py person
python3 tools/gui_follow.py off
```

These commands change the camera in the real Gazebo GUI; the noVNC browser
shows those changes live.
