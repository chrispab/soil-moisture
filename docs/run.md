You can now build and upload to a specific zone with a single command. PlatformIO will automatically set the correct IP address for the upload and the correct build flag for your code.


A) Use a virtualenv (recommended, no sudo)

python3 -m venv ~/venvs/pio
source ~/venvs/pio/bin/activate
pip install platformio
pio run -e zone1 -t upload
deactivate


To deploy to Zone 1:

bash
pio run -e zone1 -t upload
To deploy to Zone 3:

bash
pio run -e zone3 -t upload