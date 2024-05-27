# FlightController
Vague ideas about a flight controller.

## Set up

        import with STM32CubeIDE: File -> Import -> General -> File System

The code compiles without errors, activates the DMP mode of the MPU6050, it returns the quaternion and from this I estimate the values ​​for ROLL, PITCH and YAW.
For calculations use:

The code is in:
        
        .c: SingleCopter/Core/Src
        .h: SingleCopter/Core/Inc
