# Thingy:53 Aeroponic Garden

## Physical setup

## Electronics
## Attiny85
* Connect some programmer (i.e. ArduinoISP or USBasp) to Attiny85
* In platformio.ini change upload_protocol to your programmer
* You might also want to fine tune PH_MAGIC_NUMBER, TDS_A, TDS_B variables to match your sensor's characteristics
* In PlatformIO menu press "Set Fuses" in "attiny85" configuration, then click "Upload"
* Insert Attiny85 into Global Controller's shield

## Global controller setup
* Setup Ubuntu Server 20.04 (i.e via [Raspberry Pi Imager](https://www.raspberrypi.com/software/))

### Enable UART
* Disable serial console
```
sudo systemctl stop serial-getty@ttyS0.service
sudo systemctl disable serial-getty@ttyS0.service
sudo systemctl mask serial-getty@ttyS0.service
```
* Put below into /etc/udev/rules.d/10-local.rules
```
KERNEL=="ttyS0", SYMLINK+="serial0" GROUP="tty" MODE="0660"
KERNEL=="ttyAMA0", SYMLINK+="serial1" GROUP="tty" MODE="0660"
```
* Reload udev rules
```
sudo udevadm control --reload-rules && sudo udevadm trigger
```
* Add user to tty (edit parts in "<>")
```
sudo adduser <USER> tty
```
* Delete below from /boot/firmware/cmdline.txt
```
console=serial0,115200
```
* Add below to /boot/firmware/config.txt
```
dtoverlay=disable-bt
```
### Enable 1-Wire
* Add below to /boot/firmware/config.txt
```
dtoverlay=w1-gpio,gpiopin=4
```

### ROS2
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
### Mosquitto
* Install mosquitto 2.0
```
sudo add-apt-repository -y ppa:mosquitto-dev/mosquitto-ppa
sudo apt install mosquitto
sudo systemctl enable mosquitto
```

### zigbee2mqtt
* Install [zigbee2mqtt](https://www.zigbee2mqtt.io/guide/installation/01_linux.html) and set it to start as a daemon on boot. Use following config (edit parts in "<>"):
```yaml
mqtt:
  base_topic: field
  server: 'mqtt://localhost'

serial:
  port: <YOUR ZIGBEE DONGLE PORT>

advanced:
    network_key: GENERATE

external_converters:
  - fc.js
```

Create file /opt/zigbee2mqtt/data/fc.js and paste following into it:
```js
const fz = require('zigbee-herdsman-converters/converters/fromZigbee');
const tz = require('zigbee-herdsman-converters/converters/toZigbee');
const exposes = require('zigbee-herdsman-converters/lib/exposes');
const reporting = require('zigbee-herdsman-converters/lib/reporting');

const definition = {
    zigbeeModel: ['FC_v0.1'],
    model: 'FC',
    vendor: 'Nordic',
    description: 'Field Controller',
    fromZigbee: [fz.temperature, fz.humidity, fz.pressure, fz.on_off, fz.level_config],
    toZigbee: [tz.on_off, tz.level_config],
    exposes: [],
    configure: async (device, coordinatorEndpoint, logger) => {
        await reporting.bind(device.getEndpoint(16), coordinatorEndpoint, ['msTemperatureMeasurement']);
        await reporting.temperature(device.getEndpoint(16), { min: 30, max: 30, change: 0 });

        await reporting.bind(device.getEndpoint(16), coordinatorEndpoint, ['msPressureMeasurement']);
        await reporting.pressure(device.getEndpoint(16), { min: 30, max: 30, change: 0 });

        await reporting.bind(device.getEndpoint(16), coordinatorEndpoint, ['msRelativeHumidity']);
        await reporting.humidity(device.getEndpoint(16), { min: 30, max: 30, change: 0 });
    },
    endpoint: (device) => {
        return { fan: 10, led1: 11, led2: 12, strat: 13, water_valve: 14, nutri_valve: 15, sensors: 16 };
    },
};

module.exports = definition;
```

### Finishing up
* Reboot the board
```
sudo reboot now
```

## Field controller setup

## Pair field controller to global controller