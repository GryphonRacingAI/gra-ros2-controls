# ROS Jazzy Apptainer Setup

Open a terminal:

`cd apptainer`

Paste the contents of `setup_apptainer.sh` into the terminal.

This automaticallys builds the container images and places in `/local/data/<your-username>`

TODO: fix env setup issues
TODO: add build command
TODO: explain custom.def files and building it


## TODO: Flags Explanation
  - `--nv`: Enables NVIDIA GPU passthrough (requires NVIDIA drivers on host).
  - `--bind`: Mounts host directories (e.g., `--bind $HOME/workspace:/workspace` for persistent data).

## Troubleshooting
