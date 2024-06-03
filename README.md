# FlightController
Vague ideas about a flight controller.

## Set up
        git clone https://github.com/nicoRomeroCuruchet/FlightController.git

- Open **STM32CubeIDE**, browse to the folder **FlightController** and click **Launch**.
- File -> Import -> General -> **Existing Projects into Workspace**.
- In **Select root directory** browse to **SingleCopter** folder and click **Finish**.
        
The code is in:

        .c: SingleCopter/Core/Src
        .h: SingleCopter/Core/Inc

## TIMER 3 for PWM to control de motors and servos:

$$PWMFreq = \frac{Timer Clock}{(Prescaler+1)×(Counter Period+1)}$$

- PWMFreq        = 250 Hz
- TimerClock     = 84 MHz
- Prescaler      = 83
- Counter Period = 3999
