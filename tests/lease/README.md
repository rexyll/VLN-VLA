# Exclusive VLA test-environment lease

The lease workflow owns one long-running GitHub-hosted runner and installs Gazebo Harmonic plus ROS 2 Jazzy only once for that lease.

The request queue is tests/lease/request.json. A new request_id means "run another test on the same warm environment".

The lease releases after 120 minutes without a new request. A later session can start a new lease.

Supported operations:
- smoke
- p104-offline
- chassis-ground-lock
- full-robot-detachable
- base-arm-dynamics
- drinktask-e2e

GitHub-hosted runners are fresh VMs per job. Therefore the warm environment is maintained inside one long-running job rather than relying on a new workflow run to reuse an old hosted VM.
