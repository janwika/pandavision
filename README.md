# pandavision
A vision pipeline for detecting objects from RGBD images and instructing a Franka Emika Research 3 to pick them up through Simulink.
## Installation
### install Python Environment (conda)
Run this command to create an anaconda environment with the needed dependencies from the environment.yml file. Make sure that the matlabengine package in the environment.yml has the corrct version for the installed Matlab version. The version can be checked [here](https://pypi.org/project/matlabengine/#history). Anaconda can be installed [here](https://conda.io/projects/conda/en/latest/user-guide/install/index.html).
````
conda env create -f environment.yml
````
### install Intel RealSense SDK
The Intel RealSense SDK is needed for the communication with the 3D camera. Install from [this repo](https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md).
### install CUDA
For GPU acceleration CUDA needs to be installed. To install it follow [this guide](https://nvidia.github.io/cuda-python/install.html).

## Usage
### activate conda environment
Run the following command to activate the conda environment created earlier. If you chose another name for the environment replace 'pandavision' with your chosen name.
````
conda activate pandavision
````
### connect Matlab Engine
Run the following commands in the Matlab console.
````
matlab.engine.shareEngine
matlab.engine.engineName
````
The last command will return the identifier of the Matlab engine. Copy it over to config.YAML found in the home directory next to 'ENGINE_NAME' in the 'SIMULINK' section.
### Camera Calibration
To perform hand-eye calibration the robot must be active and the Simulink environment running. Make sure that the robot holds an AruCo marker with its end-effector. Also the calibration flag in one of the constant blocks in Simulink must be set to true. Also when closing the end-effector to Simulink check that the constant block responsible for opening and closing it is actually connected since in operating mode opening and closing is handled by the state flow chart.
Now, run the following command from the root directory:
````
python capture_calibration_points.py
````
The script will move the robotic arm to the predefined coordinates in ````./calibration/captures/calibration_points.py```` asking if the robot arrived after each iteration. Press enter to take a picture with the connected RealSense camera and move on to the next point. The script will terminate after the last point was reached. To calculate the calibration matrix run the following script:
````
python ./calibration/calibrate.py
````
If the program throws an error because the marker isn't visible in a picture you can enter the number of the picture to the skip array in the program itself. If too many images are corrupted, calibration won't work.
After successful calibration the calibration matrices will automatically be saved and used by the vision pipeline later, no more is needed at this point.
### Grasping objects
To grab one of the four classes (pen, knife, fork, spoon) run the following command and move through the options displayed in the command line with the arrow keys.
````
python pipeline.py
````
You will get the option to use a stock or finetuned model. The finetuned model must be located in the root directory and called 'finetuned_post_own_labels.pt'. If the stock option is used the yolov8x model will automatically be downloaded if it isn't found in the root directory.