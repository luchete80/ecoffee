

refs

https://github.com/mwood77/koffie/blob/main/koffie/koffie.ino

https://github.com/Zer0-bit/gaggiuino

https://forum.arduino.cc/t/heating-element-with-pwm-and-pid-control/545470



This uses PWM

https://github.com/shmick/Espresso-PID-Controller/blob/master/Espresso-PID-Controller.ino
uses: https://github.com/br3ttb/Arduino-PID-Library

void relayControl(void)
{
  Input = readTemps(); // Providde the PID loop with the current temperature

  if (Input >= maxBoilerTemp) // Forcibly turn off the boiler if maxBoilerTemp is reached
    digitalWrite(RelayPin, LOW);

  if (runTimeMins >= maxRunTime && operMode) // If we've reached maxRunTime, disable the PID control
    enablePID(false);

  if (operMode && Input < maxBoilerTemp)
  {
    myPID.Compute();

    // Starts a new PWM cycle every WindowSize milliseconds
    if (WindowSize < now - windowStartTime)
      windowStartTime += WindowSize;

    // Calculate the number of milliseconds that have passed in the current PWM cycle.
    // If that is less than the Output value, the relay is turned ON
    // If that is greater than (or equal to) the Output value, the relay is turned OFF.
    PWMOutput = Output * (WindowSize / 100.00);
    if ((PWMOutput > 100) && (PWMOutput > (now - windowStartTime)))
      digitalWrite(RelayPin, HIGH);
    else
      digitalWrite(RelayPin, LOW);
  }
}


This regulates pressure
https://www.home-barista.com/espresso-machines/diy-pid-koffie-project-t78030.html