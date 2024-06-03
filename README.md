# FlightController
Vague ideas about a flight controller.

## Set up
        git clone https://github.com/nicoRomeroCuruchet/FlightController.git

- Open **STM32CubeIDE** and browse to the folder **FlightController**.
- File -> Import -> General -> Existing Projects into Workspace.
- In **Select root directory** browse to SingleCopter folder and click **Finish**.
        
The code compiles without errors (checkout log.txt), activates the DMP mode of the MPU6050, it returns the quaternion and from this I estimate the values ​​for ROLL, PITCH and YAW.
For calculations use:

The code is in:
        
        .c: SingleCopter/Core/Src
        .h: SingleCopter/Core/Inc
