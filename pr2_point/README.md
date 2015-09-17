# pr2_point
Pointing gestures using the PR2's arms

## Running the service

Run in a terminal:

```
$ roslaunch pr2_point pr2_point_service.launch
```

Or, include the following line in your launch file:

```
<include file="$(find pr2_point)/pr2_point_service.launch" />
```

By default this will assume pointing with the left arm and will skip the calibration process. You can modify the parameters ```arm_index``` and ```is_calibration``` in the launch file or specify them to be different through the command line.

## Calling the service

To test the pointing, call from a terminal:

```
$ rosservice call /pr2_point_service 0.5 0.5 1.0
```

The service has thee parameters:
* ```vertical``` is a float between 0 and 1 that specifies the relative height of the target pointing gesture
* ```horizontal``` is a float between 0 and 1 that specifies the horizontal pose of the pointing gesture
* ```duration``` (in seconds) is the time spent at the pointing pose before returning to a neutral pose

(0,0) is the bottom left corner, (0,1) is the top left corner, and so on. 

For an example of how to call the service from your code, see scripts/test.py

## Calibration

Calibration involves kinestetically demonstrating a neutral pose and 4 extreme pointing poses. To calibrate run:

```
$ robot start
$ roslaunch pr2_point pr2_point_service.launch is_calibration:=true
```

Then follow the instructions. The robot's arms should get relaxed if you slightly push them and they should freeze back is you keep them constant at some pose for a few seconds. You will specify four different poses (neutral, bottom left, top left, top right, bottom right) by posing the robot's arm and pressing enter on the terminal. 

## Using two arms simultaneously

To calibrate point with both arms, specify ```arm_index:=2```. If you are calibrating, this will go through the right arm calibration first and then the left arm calibration. During normal operation, you will have two separate services for each arm ```/pr2_point_service/arm0``` and ```/pr2_point_service/arm1```. So just create __two separate service proxies__ with these service names and call the one you would like to point with.



