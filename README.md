# FlightController
Vague ideas about a flight controller.

## Set up
        git clone https://github.com/nicoRomeroCuruchet/FlightController.git

1- Open STM32CubeIDE and browse to the folder FlightController.
2 - File -> Import -> General -> Existing Projects into Workspace.
3 - In 'Select root directory' browse to SingleCopter and click Finish.
        
The code compiles without errors (checkout log.txt), activates the DMP mode of the MPU6050, it returns the quaternion and from this I estimate the values ​​for ROLL, PITCH and YAW.
For calculations use:

The code is in:
        
        .c: SingleCopter/Core/Src
        .h: SingleCopter/Core/Inc
