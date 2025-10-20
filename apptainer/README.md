**Note:** this is **ONLY** for UoL students with access to the Bragg Compute Cluster.



## ROS Jazzy Apptainer Setup 
Follow each bullet point in steps below.
>1. Open the terminal app in uni cluster (press Windows Key <kbd>⊞</kbd> and search for `terminal`). 

If you are using your own laptop go to [this section](#accessing-university-apptainer-containers-remotely-using-ssh-with-gui) to setup VPN and remotely access Bragg Computers.

Clone `gra-ros2` into dev branch and pull all git submodules
```bash
mkdir -p ~/colcon_ws/src
git clone -b dev https://github.com/GryphonRacingAI/gra-ros2.git ~/colcon_ws/src
cd ~/colcon_ws/src/apptainer
cd src; git submodule update --init --recursive; cd ..
```
>2. Run the build script to get container environment with dependencies and ros2 jazzy environment.
```bash
chmod u+x setup_apptainer.sh
sh setup_apptainer.sh
```

This automaticallys builds the container images and places in `/local/data/$USER` in 15 minutes

>3. Enter your apptainer
```bash
cd /local/data/$USER
apptainer shell --nv /local/data/$USER/ros_jazzy.sif
```
When you see the REPL below you are inside your container.

>4. Building your gra-ros2 with colcon
```bash
cd ~/colcon_ws
colcon build --symlink-install
```

```Apptainer>```
>5. Sourcing environment
```bash
source /opt/ros/jazzy/setup.bash && source ~/colcon_ws/install/setup.bash
```

You can create a file in `~` like `.fsairc` with any source or config scripts.
```bash
echo "source /opt/ros/jazzy/setup.bash && source ~/colcon_ws/install/setup.bash" > ~/.fsairc
source ~/.fsairc
```

You can now try running `ros2 launch simulation `
>6. Creating an alias to enter apptainer efficiently

Make sure you are not inside apptainer
```bash
echo "alias fsai='apptainer shell --nv /local/data/\$USER/ros_jazzy.sif'" >> ~/.bashrc
```

Now running: `fsai` should allow you to enter your apptainer container 
## Creating your Custom Apptainer Container with extra dependencies
- `cd /local/data/$USER`

That `ros_jazzy.sif` script takes around 10 minutes to complete, to add your own dependencies without waiting for 15 minutes or breaking things accidentally the `custom.def` file exists.

Familiarize yourself with containers if you haven't used containers before, if you have 
[apptainer docs](https://apptainer.org/documentation/) is self-explanatory.

>Add your custom dependencies or files in `/local/data/$USER`
- add any local files you can't extract from web eg things like installer (e.g. `ros_jazzy.def` used ZED_SDK installer, cuda key ring, .deb files etc.
- creating your own bash script e.g: `setup_username_custom.sh` is a good idea to keep track of files you added for your `custom.def`
- make sure to add to the `%files%` 
- usually done by `apt-get -y <ros-pkg-name>` in the `%runscript%` section of `custom.def`

>Build your custom container and enter it with: 
```bash
apptainer build custom.sif custom.def
apptainer shell --nv custom.sif
```
Update our `fsai` alias & create your `.fsarc` files for ease.

## Accessing University Apptainer containers remotely using SSH with GUI
Over weekends or out of uni hours you can access your containers using the method below.
>Get uol vpn: [Ivanti Secure Access Client Download](https://library.ucdavis.edu/vpn/)
- Add UOL VPN server url `<server-url>`
- Go to `feng-linux.leeds.ac.uk` on your browser
- `ssh -Y <username>@uol-pc-<id>` make sure to note the `<id>` of local pc where you installed your containers

**Note:** If above doesn't work pc must be remotely powered on by following the steps below:
```bashrc
module add wol
wol -h <ip> <mac>
```

> Continue setting things up by following the instructions [above](#ros-jazzy-apptainer-setup)

Email or message on Teams the Technical Director: `sc23pg@leeds.ac.uk` for the values of `ip`, `mac`, `id`, `<server-url>` as I am not sure I can share this in a public repo.

If too many people use the same local pc things may be slow so `sc23pg` may have to request for more: `ip`, `mac` values for a pc with `id` in Bragg Cluster.

Turning on the computer may take a few minutes, then you can `ssh` into it.

## Setting up IPG Carmaker
> 1. Build `carmaker.def`.

> 2. TODO: Setup source scripts for: `/opt/ipg/

> 3. TODO: License Setup 
## Troubleshooting
**Disk Quota Limit Exceeded**:

You shouldn't get this if you have used `/local/data/$USER` if you do let Technical Director know.
- run `quota` to check your disk quota
- run `du -sh * | sort -h` at `~` to see where disk space is being used
- carefully use `rm` to free up space, be sure to backup or important files (eg git commits)

Note: the `ros_jazzy.sif` file should only take up: 12 GB of space.
- Read more about [Disk Quotas](https://docs.redhat.com/en/documentation/red_hat_enterprise_linux/7/html/storage_administration_guide/ch-disk-quotas) or contact admin if you still run into issues or need more space.

- This is why we are using the shared disk space in `/local/data` not the NFS shared across university networks, which has ~15 GB quota per student.

