# README #

This repo hosts the software needed for the Awaiba Camera software, for the cardioscope project

### Features ###

* Grabbing camera images in real time
* Recording images (one image per frame, timestamped) in a timestamped folder
* Connecting with CTR software for camera roll compensation when the camera is attached to a robot

### How do I get set up? ###

* Configuration
Windows 7 or later (earlier might work, not tested) with visual studio 2012 (express edition is fine).

* Dependencies
Naneye cpp api (included in the repo)
Eigen lib (included in the repo)
OpenCV v>3.1.0 (versions >2.4 might work but have not been tested), compiled from source with QT Support (see http://docs.opencv.org/2.4/doc/tutorials/introduction/windows_install/windows_install.html). If directory of installation is not C:\opencv, property sheets must be changed in the project

* Deployment instructions
Compile in release mode. Copy executable file (Project_dir)/x64/Release/AwaibaOpenCV.exe in (Project_dir)/Export_executables
You can then copy the Export_executables folder to any Windows (7 or later) computer with 64 bits compatibility and Visual Studio 2012 redist executables installed. It will run out of the box.

Informations about the camera are read from the camera_info.csv file. White balance parameters, save directory for the images, as well as fixed rotation of the image (if camera is embedded in a device, for alignment) are read from this file and can be changed.

### Who do I talk to? ###

* Benoit Rosa wrote the code, with help from Junyoug Ha and George Fagogenis for the CTR part.