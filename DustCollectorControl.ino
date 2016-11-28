#include "EmonLib.h"
EnergyMonitor emon1;
EnergyMonitor emon2;
EnergyMonitor emon3;

// This is the calibration factor used to turn the variance in your ADC readings to actual amperages. There
// is some formula to calculate it, but it is much easier to just connect a device with a known power draw,
// run the arduino with debug enabled in this script, and tweak the value until the current output matches
// what you're expecting. For this application we don't need exact current values, just something in the
// general range of accurate. 
#define ICAL 52

// The number of ADC samples to use when calculating the current. At 10,000 samples/sec, 1480 should be 
// 150 ms of sampling, plenty to catch multiple cycles of a 60 hz power source. 
#define IRMS_SAMPLES 1480

// The digital pin used to trigger the dust collector relay(s)
#define RELAY_PIN 7

// LED pin to show power status
#define POWER_LED_PIN 13

// The digital interrupt pin connected to the manual override switch. The switch should be connected
// to ground as this input is configured in PULLUP mode. 
#define SWITCH_PIN 2
#define SWITCH_ON LOW
#define SWITCH_OFF HIGH

// The number of milliseconds the dust collector will continue to run after the tool has turned off.
// The default is set to 5 seconds.
#define POWER_OFF_DELAY_MS 5000

#define OFF false
#define ON true

// This is the minimum current required to turn on the tool, set this to a value
// high enough to ensure random noise won't activate the tool, but lower enough
// that all of your tools will trigger. 
#define CURRENT_THRESHOLD 2.0

// Uncomment the following line to enable analog value logging for debugging signal issues
//#define DEBUG

class Tool
{
  public:
    EnergyMonitor mon;
    int pin;

    Tool(int _pin)
    {
      pin = _pin;
      mon.current(pin, ICAL);
      
      Serial.print("New tool on pin: ");
      Serial.println(pin, DEC);
    }
};

// Configure the next section based on the number of current sensor connected to your
// Arduino, and the ADC ports used. The default project is configured for 3 tools
// connected to ports A0, A1, and A2. 

int num_tools = 3;

Tool tools[] = {
  Tool(A0),
  Tool(A1),
  Tool(A2)
};

volatile bool manualOverride = OFF;
bool powerState = OFF;
volatile unsigned long powerOffTime = 0;

void setup()
{  
  Serial.begin(9600);

  pinMode(RELAY_PIN, OUTPUT);
  pinMode(POWER_LED_PIN, OUTPUT);
  pinMode(SWITCH_PIN, INPUT_PULLUP);

  digitalWrite(RELAY_PIN, LOW);
  digitalWrite(POWER_LED_PIN, LOW);
  
  attachInterrupt(digitalPinToInterrupt(SWITCH_PIN), buttonInterrupt, CHANGE);

  // If the switch is on when we start, turn on
  if (digitalRead(SWITCH_PIN) == SWITCH_ON)
  {
    manualOverride = ON;
  }
  
  // Sleep 2 seconds on power on to let the system stabilize, I've had faulty current readings
  // immediately after power connection.
  delay(2000);
}

void loop()
{
  #ifdef DEBUG
    for (int i = 0; i < num_tools; i++)
    {
      int val = analogRead(tools[i].pin);
      double cur = tools[i].mon.calcIrms(IRMS_SAMPLES);
      Serial.print("Tool ");
      Serial.print(i + 1, DEC);
      Serial.print(" analog value: ");
      Serial.print(val, DEC);
      Serial.print(" current: ");
      Serial.println(cur, DEC);
    }
  #endif
  
  if (manualOverride == ON) 
  {
    // Only use the override switch if it is set to ON, otherwise continue to see
    // if a tool is running or not.
    changePowerState(ON);
  }
  else
  {
    bool newPowerState = OFF;
    for (int i = 0; i < num_tools && newPowerState == OFF; i++)
    {
      double current = tools[i].mon.calcIrms(IRMS_SAMPLES);
      if (current > CURRENT_THRESHOLD)
      {
        Serial.print("Tool is powered on: ");
        Serial.print(i + 1, DEC);
        Serial.print(" current: ");
        Serial.println(current, DEC);
        newPowerState = ON;
      }
    }

    changePowerState(newPowerState);
  }
}

void changePowerState(bool newPowerState)
{
  if (newPowerState == powerState)
  {
    if (powerOffTime != 0)
    {
      Serial.print("changePowerState called with state already at ");
      Serial.print(powerState, DEC);
      Serial.println(" resetting powerOffTime");
      powerOffTime = 0;

      // Turn the power LED back on as we won't be shutting down
      digitalWrite(POWER_LED_PIN, HIGH);
    }
    
    return;
  }

  if (newPowerState == OFF)
  {
    if (powerOffTime == 0) 
    {
      // This is the first call to turn off, set powerOffTime to the current time and return
      powerOffTime = millis() + POWER_OFF_DELAY_MS;
      Serial.print("Set powerOffTime to ");
      Serial.print(powerOffTime, DEC);
      Serial.print(" current time is ");
      Serial.println(millis(), DEC);

      // Turn off the LED to indicate power off sequence has begun
      digitalWrite(POWER_LED_PIN, LOW);
      
      return;
    }
    else if (powerOffTime > millis())
    {
      Serial.print("Not turning off yet, powerOffTime=");
      Serial.print(powerOffTime, DEC);
      Serial.print(" currentTime=");
      Serial.println(millis(), DEC);
      return;
    }
    else
    {
      Serial.print("powerOffTime has been reached, turning off. powerOffTime=");
      Serial.print(powerOffTime, DEC);
      Serial.print(" currentTime=");
      Serial.println(millis(), DEC);
    }
  }

  Serial.print("Changing power state to ");
  Serial.println(newPowerState, DEC);
  
  digitalWrite(RELAY_PIN, newPowerState == ON ? HIGH : LOW);
  digitalWrite(POWER_LED_PIN, newPowerState == ON ? HIGH : LOW);
  powerState = newPowerState;
  powerOffTime = 0;
}

void buttonInterrupt()
{
  if (digitalRead(SWITCH_PIN) == SWITCH_ON)
  {
    manualOverride = ON;
  }
  else
  {
    manualOverride = OFF;
    // Set the powerOffTime to the lowest possible non-zero time so the tool will turn off
    // immediately when the manual button is switched, instead of waiting for the power off
    // delay.
    powerOffTime = 1;
  }
}

