# pr2_point
Pointing gestures using the PR2's arms

## Running the service

Run in a terminal:
'''
roslaunch pr2_point pr2_point_service.launch
'''

Or, include the following line in your launch file:
'''
<include file="$(find pr2_point)/pr2_point_service.launch" />
'''

## Calling the service

To test the pointing, call from a terminal:

'''
rosservice call /pr2_point_service 0.5 0.5
'''

For an example of how to call from your code, see scripts/test.py
