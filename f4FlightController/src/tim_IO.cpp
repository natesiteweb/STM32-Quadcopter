#include "Arduino.h"
#include "tim_IO.h"

#define PPM PA0

HardwareTimer *PPMTimer;
HardwareTimer *ESC1Timer, *ESC2Timer;

uint32_t channelRising, channelFalling;
volatile uint32_t FrequencyMeasured, DutycycleMeasured, LastPeriodCapture = 0, CurrentCapture, HighStateMeasured;

volatile uint32_t rolloverCompareCount = 0;
uint32_t currentChannelPPM;

void Timers_Setup()
{
    channelRising = 1;

    switch (channelRising)
    {
    case 1:
        channelFalling = 2;
        break;
    case 2:
        channelFalling = 1;
        break;
    case 3:
        channelFalling = 4;
        break;
    case 4:
        channelFalling = 3;
        break;
    }

    PPMTimer = new HardwareTimer(TIM5);
    PPMTimer->setMode(channelRising, TIMER_INPUT_FREQ_DUTY_MEASUREMENT, PPM);

    PPMTimer->setOverflow(0xFFFF, MICROSEC_FORMAT);
    PPMTimer->attachInterrupt(channelRising, TIMINPUT_Capture_Rising_IT_callback);
    PPMTimer->attachInterrupt(channelFalling, TIMINPUT_Capture_Falling_IT_callback);
    PPMTimer->attachInterrupt(Rollover_IT_callback);
    PPMTimer->resume();

    currentChannelPPM = 0;
    input_freq = PPMTimer->getTimerClkFreq() / PPMTimer->getPrescaleFactor();

    ESC1Timer = new HardwareTimer(TIM8);
    ESC2Timer = new HardwareTimer(TIM3);

    ESC1Timer->setMode(4, TIMER_OUTPUT_COMPARE_PWM1, PC9);  //ESC1 FL
    ESC1Timer->setMode(3, TIMER_OUTPUT_COMPARE_PWM1, PC8);  //ESC2 FR
    ESC1Timer->setMode(2, TIMER_OUTPUT_COMPARE_PWM1, PC7);  //ESC3 BR
    ESC1Timer->setMode(1, TIMER_OUTPUT_COMPARE_PWM1, PC6);  //ESC4 BL

    ESC2Timer->setMode(4, TIMER_OUTPUT_COMPARE_PWM1, PB1);  //5
    ESC2Timer->setMode(3, TIMER_OUTPUT_COMPARE_PWM1, PB0);  //6
    ESC2Timer->setMode(2, TIMER_OUTPUT_COMPARE_PWM1, PA7);  //7
    ESC2Timer->setMode(1, TIMER_OUTPUT_COMPARE_PWM1, PA6);  //8

    ESC1Timer->setOverflow(5000, MICROSEC_FORMAT);//20000 - 50hz(Servo), 5000 - 200hz(ESC)
    ESC2Timer->setOverflow(5000, MICROSEC_FORMAT);

    ESC1Timer->attachInterrupt(Update_IT_callback);
    ESC1Timer->attachInterrupt(4, Compare_IT_callback);
    ESC1Timer->attachInterrupt(3, Compare_IT_callback);
    ESC1Timer->attachInterrupt(2, Compare_IT_callback);
    ESC1Timer->attachInterrupt(1, Compare_IT_callback);

    ESC2Timer->attachInterrupt(Update_IT_callback);
    ESC2Timer->attachInterrupt(4, Compare_IT_callback);
    ESC2Timer->attachInterrupt(3, Compare_IT_callback);
    ESC2Timer->attachInterrupt(2, Compare_IT_callback);
    ESC2Timer->attachInterrupt(1, Compare_IT_callback);

    ESC1Timer->resume();
    ESC2Timer->resume();

    ESC1Timer->setCaptureCompare(4, 1000);
    ESC1Timer->setCaptureCompare(3, 1000);
    ESC1Timer->setCaptureCompare(2, 1000);
    ESC1Timer->setCaptureCompare(1, 1000);

    ESC2Timer->setCaptureCompare(4, 1000);
    ESC2Timer->setCaptureCompare(3, 1000);
    ESC2Timer->setCaptureCompare(2, 1000);
    ESC2Timer->setCaptureCompare(1, 1000);
}

void Rollover_IT_callback(void)
{
    rolloverCompareCount++;

    if (rolloverCompareCount > 1)
    {
        FrequencyMeasured = 0;
        DutycycleMeasured = 0;
    }
}

void TIMINPUT_Capture_Rising_IT_callback(void)
{
    CurrentCapture = PPMTimer->getCaptureCompare(channelRising);
    /* frequency computation */

    LastPeriodCapture = CurrentCapture;
    rolloverCompareCount = 0;
}

void TIMINPUT_Capture_Falling_IT_callback(void)
{
    /* prepare DutyCycle computation */
    CurrentCapture = PPMTimer->getCaptureCompare(channelFalling);

    if (CurrentCapture > LastPeriodCapture)
    {
        frequencyRead = CurrentCapture - LastPeriodCapture;
    }
    else if (CurrentCapture <= LastPeriodCapture)
    {
        /* 0x1000 is max overflow value */
        frequencyRead = 0xFFFF + CurrentCapture - LastPeriodCapture;
    }

    if (frequencyRead > 3000)
    {
        currentChannelPPM = 0;
    }
    else
    {
        currentChannelPPM++;
    }

    if (currentChannelPPM == 1)
        frequencyRead1 = frequencyRead + 400;
    if (currentChannelPPM == 2)
        frequencyRead2 = frequencyRead + 400;
    if (currentChannelPPM == 3)
        frequencyRead3 = frequencyRead + 400;
    if (currentChannelPPM == 4)
        frequencyRead4 = frequencyRead + 400;
    if (currentChannelPPM == 5)
        frequencyRead5 = frequencyRead + 400;
    if (currentChannelPPM == 6)
        frequencyRead6 = frequencyRead + 400;
}

void Update_IT_callback(void)
{ // Update event correspond to Rising edge of PWM when configured in PWM1 mode
  //digitalWrite(pin2, LOW); // pin2 will be complementary to pin
}

void Compare_IT_callback(void)
{ // Compare match event correspond to falling edge of PWM when configured in PWM1 mode
  //digitalWrite(pin2, HIGH);
}