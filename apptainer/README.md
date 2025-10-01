# ROS Jazzy Apptainer Setup
Open a terminal:

`cd apptainer`

Paste the contents of `setup_apptainer.sh` into the terminal.

This automaticallys builds the container images and places in `/local/data/<your-username>`

Now everytime you want to use the environment you can enter a shell by running:
`apptainer shell --nv /local/data/<your-username>/ros_jazzy.sif`

Now you can run your `ros2` commands and should be able to start contributing to the codebase and working with the simulation.

When you see the prompt below: 

```Apptainer>```

You are in your apptainer environment.

Note that you should paste the contents in `%runscript` into ```Apptainer>``` in order to get ros2 sourced.

You can attach your apptainer to vscode using this [guide](https://github.com/microsoft/vscode-remote-release/issues/3066#issuecomment-1019500216)

## Custom def file
Running the script will also create a `custom.def` file which sources the `ros_jazzy.sif` container to allow you to install your own custom packages.

Build the custom container with: `apptainer build custom.sif custom.def`


## Flags Explanation
  - `--nv`: Enables NVIDIA GPU passthrough (requires NVIDIA drivers on host).
  - `--bind`: Mounts host directories (e.g., `--bind $HOME/workspace:/workspace` for persistent data).

## Accessing University Apptainer containers remotely using SSH with GUI
TODO: 

## Troubleshooting
**Disk Quota Limit Exceeded**:
- run `quota` to check your disk quota
- run `du -sh * | sort -h` at `~` to see where disk space is being used
- carefully use `rm` to free up space, be sure to backup or important files (eg git commits)

Note: the `ros_jazzy.sif` file should only take up: 12 GB of space.
- Read more about [Disk Quotas](https://docs.redhat.com/en/documentation/red_hat_enterprise_linux/7/html/storage_administration_guide/ch-disk-quotas) or contact admin if you still run into issues or need more space.

- This is why we are using the shared disk space in `/local/data` not the NFS shared across university networks, which has ~15 GB quota per student.

