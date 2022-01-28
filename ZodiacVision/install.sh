sudo apt install python3-pip
python3 -m pip install -U pip
python3 -m pip install pynetworktables flask_opencv_streamer
if [ ! -d "/usr/local/zed" ]; then
    wget -nc -O ZED_SDK_Tegra_JP45_v3.4.2.run https://download.stereolabs.com/zedsdk/3.4/jp45/jetsons
    chmod +x ZED_SDK_Tegra_JP45_v3.4.2.run
    ./ZED_SDK_Tegra_JP45_v3.4.2.run -- silent
fi
