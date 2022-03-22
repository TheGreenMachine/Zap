Notes for installing on Windows

* Install python 3.9 locally and create a virtual environment using venv
* Follow the instructions at https://
* pip install flask_opencv_streamer

Note: the zed python installer does not install the distance module properly so copy the 2 files below to the pyzed
virtual lib location i.e. Zap\venv\Lib\site-packages\pyzed
* "C:\Program Files (x86)\ZED SDK\bin\sl_zed64.dll"
* "C:\Program Files (x86)\ZED SDK\bin\sl_ai64.dll"
