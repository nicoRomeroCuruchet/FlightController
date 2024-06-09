# FlightController
Vague ideas about a flight controller.

## Set up
        git clone https://github.com/nicoRomeroCuruchet/FlightController.git

- Open **STM32CubeIDE**, browse to the folder **FlightController** and click **Launch**.
- File -> Import -> General -> **Existing Projects into Workspace**.
- In **Select root directory** browse to **SingleCopter** folder and click **Finish**.
        
For the details in the code check:

        .c: SingleCopter/Core/Src
        .h: SingleCopter/Core/Inc

### TIMER 3 for PWM to control de motors and servos:

$$PWMFreq = \frac{TimerClock}{(Prescaler+1)×(Counter Period+1)}$$

- PWMFreq        = 250 Hz
- TimerClock     = 84 MHz
- Prescaler      = 83
- Counter Period = 3999

### TIMER 1 for PWM input capture for read the radio controller:

- PWM freq of the radio = 50 Hz (20 ms period)
- Duty Cycle is in the range [1000, 2000] microseconds

##### PWM input capture configuration:
- Clock Source: **Internal Clock**
- Channel x: Input capture direct mode
- Prescaler = 83
- Counter Mode: up
- Counter Period = 19999
- Internal Clock Division: No Division
- Repetition Counter: 0
- auto-reload preload: enable
- PWM freq 50 Hz = 84Mhz / (84 * 20000)

   
