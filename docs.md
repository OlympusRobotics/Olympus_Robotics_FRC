# Documentation for robot

## installation and updating
```
- go to wpilib, download the newest iso. Mount and execute.
- ./gradlew build
```

## Phoenix Dependencies
```
- in vscode overview, under 3rd party libs find the phoenix maven link - go to cmd palette vscode - Manage Vendor Libraries - install new libraries online - enter link
```

## output
Type riolog into command palette
```
// both print text
std::cout << "text" << endl;
fmt::print("text {}\n", some_variable);
// the fmt print needs the some_variable after it to work for some reason
```

## installing libraries
```
https://store.ctr-electronics.com/documentation
```

## xBox controller mapping
```
hw.xBox.GetXButton() = left trigger
hw.xBox.GetYButton() = right trigger
```

# Control logic 2022
Robot.cpp is the main file.
    - Hardware map creates objects of all motors, hid devices, pneumatics, etc.
    - Normal driving
        - Driving is proportionally powered by the y axis of the right joystick
        - Turning is capped at the sens variable for better driving.
    - Drift mode
        - Allows for turning while driving
        - Twisting the joystick sets one side of the wheels to full power while the other stays proportional - left twist for left side, etc 
