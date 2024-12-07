# takeoff_and_land
mimicked [MAVSDK takeoff_and_land sample](https://mavsdk.mavlink.io/main/en/cpp/examples/takeoff_and_land.html)  without MAVSDK library modifying  [C-UART Interface Example](https://github.com/mavlink/c_uart_interface_example)

Building
========

```
$ make
```

Hardware Setup
=========

Connect the USB programming cable to your Pixhawk.  

If you want to be able to interact with this example in Pixhawk's NuttX shell, you'll need a Telemetry Radio or an FTDI developer's cable.  See the Exploration section below for more detail.

Also Note: Using a UART (serial) connection should be preferred over using the USB port for flying systems.  The reason being that the driver for the USB port is much more complicated, so the UART is a much more trusted port for flight-critical functions.  To learn how this works though the USB port will be fine and instructive.

Execution
=========

You have to pick a port name, try searching for it with 
```

$ ls /dev/ttyACM* 
$ ls /dev/ttyUSB*
```

Alternatively, plug in Pixhawk USB cable again and issue the command:
```
$ dmesg
```
The device described at the bottom of dmesg's output will be the port on which the Pixhawk is mounted. 

The Pixhawk USB port will show up on a `ttyACM*`, an FTDI cable will show up on a `ttyUSB*`.


Run the example executable on the host shell:

```
$ ./mavlink_control -d /dev/ttyACM0
```

To stop the program, use the key sequence `Ctrl-C`.


Simulation
===========

There is also the possibility to connect this example to the simulator using:

```
$ ./mavlink_control -u 127.0.0.1 -a
```
The -a argument enables arming, takeoff and landing of the copter. Use this argument with care on a real copter!
