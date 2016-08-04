# mouse_as_joystick
Wrapper to read from a mouse (e.g. bluetooth mouse) and publish ros messages to the "/ada/joy" topic. Note that this package will disable the device from providing inputs as a mouse.

## Usage
The package will attempt to guess which connected device is the mouse. You can try the most automatic mode with:

```
rosrun mouse_as_joystick MouseToRosMessage.py
```

If that doesn't work, you may need to manually specify the device. To do so, in a terminal, run:

```
$ xinput
```

And node the id number of the device you want to connect to. Now you can specify this to the package with 

```
rosrun mouse_as_joystick MouseToRosMessage.py -id=ID
```

You may also need to specify the mouse index number. This is given by X in /dev/input/mouseX. You can then specify this with

```
rosrun mouse_as_joystick MouseToRosMessage.py -id=ID -num=NUM
```
