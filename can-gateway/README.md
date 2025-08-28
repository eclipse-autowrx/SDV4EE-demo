# IDD-motor CAN/Ethernet Gateway to the vehicle central computer

## Building for Zephyr
To build freestanding Zephyr applications, refer the [blog article](https://www.zephyrproject.org/how-to-build-your-zephyr-app-in-a-standalone-folder/).
The application have been tested with zephyr 4.1.99.0 SDK 0.17.0

after Zephyr installation source the Zephyr dependencies in the terminal:
```
$ source ~/zephyrproject/.venv/bin/activate
$ source ~/zephyrproject/zephyr/zephyr-env.sh
```

Subsequently, you can build the application using _west_. We have integrated the build dependencies of theapplication into the main CMake of the project. Hence build commands are executed from the main folder of the repository.
```
$ west build -b arduino_portenta_h7/stm32h747xx/m7  .
$ west build -t run # OR west build -t flash
```

The parameters of the application (e.g. gateway IP ) can be set in the [prj.conf.](prj.conf)

