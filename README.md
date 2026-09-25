# Steelhead AUV

This repository contains the ROS2 system for the UBC SubBots Steelhead AUV. It is meant to be launched in Ubuntu 26.04 on the Radxa X4 on board the Steelhead AUV. The ROS2 Foxy Ubuntu 20.04 is now archived as a seperate branch labelled `foxy-20.04`, as the migration to Ubuntu 26.04 brought a lot of breaking changes.

# Contents

- [Setup](#setup)
    - [Workspace Sourcing](#workspace-sourcing)
    - [OpenCV Installation](#opencv-installation)
    - [ROS2 Dependencies](#ros2-dependencies)
- [Development Setup](#development-setup)
- [Contributing](#contributing)
- [AI Policy](#ai-policy)
- [Tips](#tips)
- [Useful Shortcuts](#useful-shortcuts)

## Setup
This guide assumes you have already followed the guide for installing Ubuntu, ROS2, and Gazebo found in the [wiki](https://github.com/ubc-subbots/steelhead/wiki/Installation-Guide).

To get started, first clone this repo to your computer running Ubuntu 26.04 into whatever directory you choose by running

    git clone https://github.com/ubc-subbots/steelhead.git

### Workspace Sourcing
Next, we need to add the setup script to our `.bashrc` so that it is sourced on every new terminal. Open up `~/.bashrc` in a text editor or in nano as such

    nano ~/.bashrc

Append the following lines to the bottom of the file

    source /opt/ros/lyrical/setup.bash                        # global setup script
    source <PATH_TO_STEELHEAD>/steelhead/install/setup.bash   # local setup script
    export RCUTILS_COLORIZED_OUTPUT=1

Now your workspace is sourced on every terminal.

### OpenCV Installation
OpenCV is available in the Ubuntu package manager. It may be already installed, but you can install it along with its Python bindings simply by running:

    sudo apt-get update
    sudo apt-get install -y libopencv-dev python3-opencv

OpenCV is now successfully installed!

### ROS2 Dependencies
Source the global ROS2 setup script in the terminal (or just start a new terminal since we already added it to our `~/.bashrc` file)

    source /opt/ros/lyrical/setup.bash
  
Next, from the folder `steelhead`, install rosdep as such
 
    cd <PATH_TO_STEELHEAD>
    sudo apt install python3-rosdep
    sudo rosdep init
    rosdep update
  
Then, resolve any dependency issues using the following commands
 
    sudo apt install python3-pip
    PIP_BREAK_SYSTEM_PACKAGES=1 rosdep install -i --from-path src --rosdistro lyrical -y -r
  
If any packages fail to install, try to manually install them with 

    pip install <PACKAGE_NAME>

From the same folder, build all the packages using the following command

    colcon build
    
Once this is done, open a new terminal for the `.bashrc` to be executed and the required scripts be sourced. To perform a sanity check that everything is working, launch the Gazebo sim by launching

    ros2 launch steelhead_bringup barebones_gazebo_launch.py
   
If this command executes successfully, you are ready to develop! If the simulation is incredibly laggy, check out the [optimization guide](./src/steelhead_gazebo/README.md#optimization-guide). 

Now that you're set up, check out [The Challenge](https://github.com/ubc-subbots/steelhead/wiki/The-Challenge) page for your first task!
    
## Development Setup
We officially support VSCode-like IDEs (VSCode, Cursor), so if you're really cool and use Neovim, keep in mind that we may not be able to help with every question. These are technically optional but make life easier and keeps our codebase consistent.

### C++ Tooling (clangd)
This package uses clangd for C++ language support (i.e., go-to-definition, autocomplete, error highlighting). To set it up:
1. Install clangd:

        sudo apt install clangd
2. Install the clangd VS Code extension (By LLVM). When prompted, disable the Microsoft C++ IntelliSense Engine.
3. Install jq via:

        sudo apt install jq

4. Build the workspace with compile commands export enabled:
        
        colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

5. Merge the per-package compile commands into the workspace root:

        jq -s 'map(.[])' build/*/compile_commands.json > compile_commands.json

Re-run steps 4-5 after adding new packages or source files, or use the clang alias detailed below.

### Python Tooling (Ruff)
We recommend using Ruff for Python linting/formatting. To get it formatting on save, simply download the Ruff vscode extension and add this to your `settings.json`:

```
"[python]": {
    "editor.defaultFormatter": "charliermarsh.ruff",
    "editor.formatOnSave": true,
    "editor.codeActionsOnSave": {
      "source.fixAll.ruff": "explicit",
      "source.organizeImports.ruff": "explicit"
    }
  }
```

## Contributing
To learn how to contribute to this repo, see the seperate [workflow](WORKFLOW.md) and [conventions](CONVENTIONS.md) documents.

## AI Policy
While AI is incredibly useful for development and we do encourage the use of it, we ask you to please understand and follow the following policies:

1. **Understand what you are putting out.** If you are making contributions to the codebase, make sure you understand your changes. For example, if a general question about its functionality is asked, you should be able to answer it. To a lesser extent, you'll be doing yourself a disfavor by hampering your learning.
2. **Be responsible for your code.** Similar to the last point, you are responsible for any code that you do contribute. If something breaks, own up to it and understand what went wrong. We're not going to be mad at mistakes, but "claude did it" isn't the greatest explanation. This is especially important because AI often gets things wrong in ROS2/Gazebo because the amount revisions it has, so it often pulls from outdated sources.
3. **Use and add to the provided skills/agents.** We're starting to add more of them, so make sure to use them such that each agent is equipped with the same set of tools.
4. **Be smart about it.** If you are requesting a review from a human, make sure to put actual human effort in it. For example, don't commit a comment like "// Add the following code snippet to your code!". 
    
## Tips
Here are some tips to be aware of when developing on this repository and when developing in ROS2 in general
- Make sure when you run any `colcon` command such as `colcon build` or `colcon test` that you do so in the root folder of this directory (i.e `steelhead`)
- After creating any new component nodes, you must either source the local setup script or simply open up a new terminal for them to show up under the command `ros2 component types` and be usable by the pipeline.
- Make sure you spell topics/services/actions correctly, be sure to debug by using `ros2 topic|service|action list`and `rqt_graph` to see that you are using the desired communication channels.
- If you have added a dependency to a package by modifying the appropriate files (`CMakeLists.txt`, `package.xml`) and the build of that package is failing because it says it can't find the package, make sure you have it installed by running `rosdep install -i --from-path src --rosdistro lyrical -y` in the `steelhead` folder, and also that a release for the distro we are using (`lyrical`) exists on the ROS2 package index.
- For non-ROS2 dependencies, check [here](https://github.com/ros/rosdistro/tree/master/rosdep) to see the available system dependencies that can be used with `rosdep`.

## Useful Shortcuts
If you'd like, you can add these aliases to the bottom of your `.bashrc` to make common commands just a bit easier to remember:

```
alias setup='nano ~/.bashrc'
alias build='colcon build && source install/setup.bash' # clean build 
alias clang='colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON && jq -s "map(.[])" build/*/compile_commands.json > compile_commands.json && source install/setup.bash'
alias clean='rm -r build install log' # cleans the workspace (MAKE SURE THAT YOU ONLY USE THIS IN THE BASE OF STEELHEAD)
alias run='ros2 launch steelhead_bringup barebones_gazebo_launch.py'
```
