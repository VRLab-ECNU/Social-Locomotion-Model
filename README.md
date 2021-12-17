# Social-Locomotion-Model
This project shows how human and robot locomotion paths can be generated by the social locomotion model.

## System requirements
1. All code has been tested on Mac OS (Version 10.10-15,11,12)
2. All modelling code was tested with Matlab R2019a.
3. All data analysis and graphing code was tested with R (Version 3.4.4).
4. The Fast Marching Toolbox need to be installed（Toolbox Fast Marching：Version 1.2.0.0 by Gabriel Peyre, https://www.mathworks.com/matlabcentral/fileexchange/6110-toolbox-fast-marching)

## Installation guide
No installation is required before running the code and install time is not applicable here. You only need to download the project and set the working directory to current folder.

## Demo
The Demo shows how the social locomotion model generated the path by taking the position and orientation of a human as inputs.
The output path is generated in an (X,Y) format with the unit of centimeter and the computation usually takes around 0.5 second with a map width of 5 meters.

## Instructions for use
Data for replicating the results in the manuscript are available at http://doi.org/10.6084/m9.figshare.17250029
You can download the data folder of each experiment to the same directory as the corresponding experiment-specified code and run code to get results described in the manuscript.