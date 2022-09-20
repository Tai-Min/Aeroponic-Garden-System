# Thingy:53 Aeroponic Garden

## Physical setup

## Electronics
### Attiny85
* Connect some programmer (i.e. ArduinoISP or USBasp) to Attiny85
* In platformio.ini change upload_protocol to your programmer
* In PlatformIO menu press "Set Fuses" in "attiny85" configuration, then click "Upload"
* Insert Attiny85 into Global Controller's shield

## Global controller setup
* Setup Ubuntu Server 20.04 (i.e via [Raspberry Pi Imager](https://www.raspberrypi.com/software/))
* Install [ROS2 Foxy Base version](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html)
* Clone this repo to your home directory (/home/$USER/TODO)
* Build ROS components:
```bash
cd ~/TODO/ros_workspace && colcon build
```
* Place ROS setups in your .bashrc (edit parts in "<>")
```
source /opt/ros/foxy/setup.bash
source /home/<USER>/TODO/ros_workspace/install/setup.bash
```
* Install mosquitto 2.0
```
sudo add-apt-repository -y ppa:mosquitto-dev/mosquitto-ppa
sudo apt install mosquitto
sudo systemctl enable mosquitto
```
* Install [zigbee2mqtt](https://www.zigbee2mqtt.io/guide/installation/01_linux.html) and set it to start as a daemon on boot. Use following config (edit parts in "<>"):
```yaml
mqtt:
  base_topic: field
  server: 'mqtt://localhost'

serial:
  port: <YOUR ZIGBEE DONGLE PORT>

advanced:
    network_key: GENERATE
```
* Reboot the board

## Field controller setup

## Pair field controller to global controller