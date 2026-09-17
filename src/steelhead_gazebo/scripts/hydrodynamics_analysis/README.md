# Model Analysis for Simulation

This repository was design to analyze the Steelhead robot and extract certain parameters required for simulation, mostly related to the hydrodynamics model. These parameters can be set in the models `.sdf` file under the hydrodynamics tag.

These instructions were made on Ubuntu 26.04 since that is what Steelhead runs, and may change throughout versions.

If you are using this on a new robot, hello, how is life? I hope this process hasn't been too difficult thus far, because it certainly was for me. I'll try to make this process a bit better for you, so follow these instructions:

0. Create a virtual environment and activate it through `python3 -m venv .venv && source .venv/bin/activate`.
1. Run `pip install -r requirements.txt` to install requirements.
2. For an added mass matrix, we need a simpler model to analyze since the calculations are arduous. This can be done using the FeatureWorks feature in solid works, or you can model a super simplified version of the base robot like I did. Attached is an example blender file.
	* If you get an error downloading Capytaine, it may be because you lack the required builder. Run `sudo apt install gfortran build-essential`.
3. Change the top level variables to the appropriate file names and run `python3 calculate_parameters.py`. This will do the calculations and print out the added mass matrix, center of buoyancy, and volume which you can copy into the `.sdf` file.

To be clear, these numbers are approximations only. That being said, it's a simulation, so the whole thing in general is an approximation.

# Citation
Berg, 2012 Development and Commissioning of a DP system for ROV SF 30k https://ntnuopen.ntnu.no/ntnu-xmlui/handle/11250/238170
