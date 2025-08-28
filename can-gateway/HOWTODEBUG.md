# Setup
## USB Binding
### Windows
Download and install [usbipd]. 
### WSL
```
sudo apt update -y
sudo apt upgrade -y
sudo apt install linux-tools-virtual hwdata
sudo update-alternatives --install /usr/local/bin/usbip usbip $(command -v ls /usr/lib/linux-tools/*/usbip | tail -n1) 20
sudo apt-get install minicom`
```
## Debugger
All the following things should happen in WSL:
Ubuntu installations:
```
sudo apt install gdb-multiarch
sudo apt-get install stlink-tools
sudo apt-get install openocd
```
VSCode Extensions:
```
code --install-extension webfreak.debug
code --install-extension marus25.cortex-debug
code --install-extension jeandudey.cortex-debug-dp-stm32h7
```

# Debugging
## Pre-requisite
In Windows: Bind USB:
1. Find the BUSID for `ST-Link Debug`:
```
usbipd list
```
2. Bind the port (only needs to be done once)
```
usbipd bind --busid <BUSID>
```
3. Attach WSL
```
usbipd attach --wsl --busid <BUSID>
```
In WSL:
1. Check that the device is there:
```
lsusb
```
2. Then:
```
sudo chmod +777 /dev/ttyACM0
```

## Start Debug session
1. Build a debug version, then flash it
### V1
Execute the start_gdb.sh script
```

```
### V2
```
west debug --runner openocd
```
