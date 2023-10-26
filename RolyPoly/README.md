# How to use the Grasshopper assembly
You should follow the Grasshopper installation instructions found in the main readme.

This plugin can generate programs for one or more Create3 robots(no limit on the number of robots). 

Programs consist of several simple classes:
1. "moves" = the entire program array
    1. "sequence step" = a point in time when all robots in the scene execute their current task simultaneously, driving, singing, waiting, rotating, etc.
        1. "messages" = for a single robot, an action it should perform

## This assembly allows two key functionalities:
1. From simple curves/geometry in the Rhino viewport and action-generating components in the Grasshopper window, it generates raw text that represents a series of robot actions, which can be copied into our other ros2 workspace (see ../ros2_ws)
1. From the above raw text, the plugin can preview the moves a robot makes, allowing you to validate your changes prior to sending them to the robot. This includes optional preview geometry so you can see how any payload on top of the robot will move / conflict with other robots.

## Beyond this, there are several ways you can interact with our tools:
1. You can draw polycurves in the Rhino viewport (curves that are created by drawing individual lines and arcs in the Rhino viewport and "joining" them together), and use them to create a series of actions.
1. You can create simple actions in Grasshopper, without curves in the Rhino viewport, and string them together into sequences
1. You can create complex sequences that are combinations of polycurve moves and simple actions.
1. You can create complex scenes with multiple robots, the limit being how many Create3 robots you own...
1. Our example scripts provide additional information about how to use the tools.

## Limitations:
1. For the "polycurve" inputs, closed curves (curves with start and end points in the same place) are not supported.
1. Make sure polycurves and robot direction lines are drawn within the XY plane.
1. The software is generally unit agnostic (it internally converts units to meters), but if you change model units after you start the model, you will need to hit "recompute" in Grasshopper for it to propagate the unit changes.
1. We cannot currently send XY coordinate goals to the Create3. We attempted to do this but there are too many variables that make this buggy:
    1. XY coordinates depend on the Create3's internal coordinate system.
    1. This internal coordinate system changes every time the robot is turned on
    1. There is a way to reset the odometry on the Create3, to define your own coordinates, but doing so seems to sometimes put the Create3 into an error state that forces you to restart it.
    1. If you figure out how to make this work, let us know!
1. Direct wheel drive commands are not supported. Current actions are "goals" for the robot to complete.
    1. We are interested in developing this functionality but doing so will require developing scripts that monitor the odometry of the robot while driving and deciding when to stop, etc.
1. While it would be amazing to directly send commands from Grasshopper to the robot, this is not currently possible.