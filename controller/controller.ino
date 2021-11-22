#include <transceiver.h>
#include <LiquidCrystal_I2C.h>
#include <Wire.h>

// LEDs pins
#define CONTROLLED_LED_PIN    0
#define MANUALDEFL_LED_PIN    1
#define RADIO_LED_PIN         2

#define PITCH_LED_PIN         22
#define P_GAIN_LED_PIN        21
#define I_GAIN_LED_PIN        20
#define D_GAIN_LED_PIN        17

// Potentiometer pin
#define POT_CONTROL_PIN       23

// Buttons pins
#define CONTROLLED_BTN_PIN    14
#define MANUALDEFL_BTN_PIN    15
#define NEUTRAL_BTN_PIN       16

// Controlled Mode Input pins
#define PITCH_INPUT_PIN       5
#define P_GAIN_INPUT_PIN      6
#define I_GAIN_INPUT_PIN      7
#define D_GAIN_INPUT_PIN      8

// LCD I2C channel
#define LCD_I2C_ADDR      0x3F

#define RADIO_TX_ADDRESS     96
#define RADIO_RX_ADDRESS     69

LiquidCrystal_I2C lcd(LCD_I2C_ADDR, 20, 4);

// Singleton instance of the radio driver
RH_RF69 rf69(RFM69_CS, RFM69_INT);

// Class to manage message delivery and receipt
RHReliableDatagram rf69_manager(rf69, RADIO_TX_ADDRESS);

// -1 - unset
// 0 - controlled
// 1 - neutral
// 2 - manual deflection
int mode = -1;

// -1 - unset
// 0 - pitch angle
// 1 - P gain
// 2 - I gain
// 3 - D gain
int controlledModeInput = -1;

double servoOffset = 90;

float deflAngle = servoOffset;
float refPitchAngle = 0;
float currentPitchAngle = 0;
float Kp = 1;
float Ki = 0;
float Kd = 0;

float potVal = 0;
float potValTolerance = 2;

float lastTimeTaken = micros()/1.0E6;
double radioInterval = 5;

void setLcdData(float deflAngle, float pitchAngle, float refPitchAngle, float Kp, float Ki, float Kd) {
  // display the angles and the gains on the lcd
  lcd.clear();
  
  lcd.setCursor(0, 0);
  lcd.print("Defl  ");
  lcd.print(deflAngle - servoOffset);
  
  lcd.setCursor(0, 1); 
  lcd.print("Pitch "); 
  lcd.print(pitchAngle);

  lcd.setCursor(0, 3);
  lcd.print("Ref Pitch "); 
  lcd.print(refPitchAngle);

  lcd.setCursor(12, 0);
  lcd.print("Kp "); 
  lcd.print(Kp);

  lcd.setCursor(12, 1);
  lcd.print("Ki "); 
  lcd.print(Ki);

  lcd.setCursor(12, 2);
  lcd.print("Kd "); 
  lcd.print(Kd);
}

void setup() 
{
  Serial.begin(115200);
  
  // initialize the lcd 
  lcd.init();                     
  lcd.backlight();

  // display default angles and gains
  setLcdData(deflAngle, currentPitchAngle, refPitchAngle, Kp, Ki, Kd);
  
  // initialize the radio
  transceiverSetup(rf69, rf69_manager);

  // set pin modes
  pinMode(CONTROLLED_LED_PIN, OUTPUT);
  pinMode(MANUALDEFL_LED_PIN, OUTPUT);

  pinMode(PITCH_LED_PIN, OUTPUT);
  pinMode(P_GAIN_LED_PIN, OUTPUT);
  pinMode(I_GAIN_LED_PIN, OUTPUT);
  pinMode(D_GAIN_LED_PIN, OUTPUT);

  pinMode(CONTROLLED_BTN_PIN, INPUT);
  pinMode(MANUALDEFL_BTN_PIN, INPUT);
  pinMode(NEUTRAL_BTN_PIN, INPUT);

  pinMode(PITCH_INPUT_PIN, INPUT);
  pinMode(P_GAIN_INPUT_PIN, INPUT);
  pinMode(I_GAIN_INPUT_PIN, INPUT);
  pinMode(D_GAIN_INPUT_PIN, INPUT);
  
  // turn the LEDs off
  statusLEDs(false, false, false);
  inputLEDs(false, false, false, false);
}

void loop() 
{ 
  // listen for pitch & deflection angles every 1s

  if (mode == 0) radioInterval = 5;
  if (mode == 2) radioInterval = 1;

  if (lastTimeTaken + radioInterval <= micros()/1.0E6)
  {
    String cmd = "Demand angles";

    String response = transmit(rf69, rf69_manager, RADIO_RX_ADDRESS, cmd);

    if (response.indexOf("Angles received") != -1) 
    {
      Serial.println("ANGLES RECEIVED");
      Serial.println(response);
  
      // blink Radio status LED
      statusLEDBlink(RADIO_LED_PIN);
  
      // update the parameters
      currentPitchAngle = getNumberFromString(response, "Pitch Angle: ");
      Serial.println(currentPitchAngle);
  
      deflAngle = getNumberFromString(response, "Defl Angle: ");
      Serial.println(deflAngle);
  
      setLcdData(deflAngle, currentPitchAngle, refPitchAngle, Kp, Ki, Kd);
    }
    
    lastTimeTaken = micros()/1.0E6;     
  }

  // get the current mode from analog input responses
  int currentMode = getCurrentMode();

  if (currentMode == -1) 
  {
    currentMode = mode;
  }

  // gets the current Input (pitch, P, I or D)
  int currentControlledModeInput = getControlledModeInput();

  if (currentControlledModeInput == -1) 
  {
    currentControlledModeInput = controlledModeInput;
  }
  
  // If on controlled mode
  if (currentMode == 0)
  {
    controlledModeInput = currentControlledModeInput;

    // If was not on controlled mode before
    if (mode != 0) 
    {
      // Transmit the instructions to activate a Controlled mode on aircraft
      String cmd = "Controlled";
  
      String response = transmit(rf69, rf69_manager, RADIO_RX_ADDRESS, cmd);
      
      if (response.indexOf("Controlled Activated!") != -1) 
      {
        // update the parameters
        currentPitchAngle = getNumberFromString(response, "Pitch Angle: ");
        deflAngle = getNumberFromString(response, "Defl Angle: ");

        // set gains to default values
        Kp = 1;
        Kd = 0;
        Ki = 0;
        
        // switch Controlled mode status LED ON
        // switch Manual defl. mode status LED OFF
        statusLEDs(true, false, false);

        setLcdData(deflAngle, currentPitchAngle, refPitchAngle, Kp, Ki, Kd);
      }
    }

    // set the mode
    mode = currentMode;

    if (controlledModeInput == 0) // if on Controlled and Pitch input activated
    {
      float potValPrevious = potVal;

      potVal = getCurrentPotValue();

      if (potVal > potValPrevious + potValTolerance || potVal < potValPrevious - potValTolerance)
      {
        // set the reference Pitch angle and send it to the aircraft
        refPitchAngle = mapPotVal(potVal, "Pitch");

        String cmd = "Ref Pitch: " + String(refPitchAngle);
    
        String response = transmit(rf69, rf69_manager, RADIO_RX_ADDRESS, cmd);
        
        if (response.indexOf("Ref Pitch: ") != -1) 
        {
          // update the parameters
          currentPitchAngle = getNumberFromString(response, "Pitch Angle: ");
          deflAngle = getNumberFromString(response, "Defl Angle: ");
          
          // blink Controlled mode status LED
          statusLEDBlink(CONTROLLED_LED_PIN);

          setLcdData(deflAngle, currentPitchAngle, refPitchAngle, Kp, Ki, Kd);
        }
      }

      // turn the Pitch LED ON
      inputLEDs(true, false, false, false);
    }
    else if (controlledModeInput == 1) // if on Controlled and P Gain input activated
    {
      float potValPrevious = potVal;

      potVal = getCurrentPotValue();

      if (potVal > potValPrevious + potValTolerance || potVal < potValPrevious - potValTolerance)
      {
        // set the P Gain value and send it to the aircraft
        Kp = mapPotVal(potVal, "PGain");

        String cmd = "Kp: " + String(Kp);
    
        String response = transmit(rf69, rf69_manager, RADIO_RX_ADDRESS, cmd);
        
        if (response.indexOf("Kp: ") != -1) 
        {
          // update the parameters
          currentPitchAngle = getNumberFromString(response, "Pitch Angle: ");
          deflAngle = getNumberFromString(response, "Defl Angle: ");
          
          // blink Controlled mode status LED
          statusLEDBlink(CONTROLLED_LED_PIN);

          setLcdData(deflAngle, currentPitchAngle, refPitchAngle, Kp, Ki, Kd);
        }
      }

      // turn the Kp LED ON
      inputLEDs(false, true, false, false);
    }
    else if (controlledModeInput == 2) // if on Controlled and I Gain input activated
    {
      float potValPrevious = potVal;

      potVal = getCurrentPotValue();

      if (potVal > potValPrevious + potValTolerance || potVal < potValPrevious - potValTolerance)
      {
        // set the I Gain value and send it to the aircraft
        Ki = mapPotVal(potVal, "IGain");

        String cmd = "Ki: " + String(Ki);
    
        String response = transmit(rf69, rf69_manager, RADIO_RX_ADDRESS, cmd);
        
        if (response.indexOf("Ki: ") != -1) 
        {
          // update the parameters
          currentPitchAngle = getNumberFromString(response, "Pitch Angle: ");
          deflAngle = getNumberFromString(response, "Defl Angle: ");
      
          // blink Controlled mode status LED
          statusLEDBlink(CONTROLLED_LED_PIN);

          setLcdData(deflAngle, currentPitchAngle, refPitchAngle, Kp, Ki, Kd);
        }
      }

      // turn the Ki LED ON
      inputLEDs(false, false, true, false);
    }
    else if (controlledModeInput == 3) // if on Controlled and D Gain input activated
    {
      float potValPrevious = potVal;

      potVal = getCurrentPotValue();

      if (potVal > potValPrevious + potValTolerance || potVal < potValPrevious - potValTolerance)
      {
        // set the D Gain value and send it to the aircraft
        Kd = mapPotVal(potVal, "DGain");

        String cmd = "Kd: " + String(Kd);
    
        String response = transmit(rf69, rf69_manager, RADIO_RX_ADDRESS, cmd);
        
        if (response.indexOf("Kd: ") != -1) 
        {
          // update the parameters
          currentPitchAngle = getNumberFromString(response, "Pitch Angle: ");
          deflAngle = getNumberFromString(response, "Defl Angle: ");
          
          // blink Controlled mode status LED
          statusLEDBlink(CONTROLLED_LED_PIN);

          setLcdData(deflAngle, currentPitchAngle, refPitchAngle, Kp, Ki, Kd);
        }
      }

      // turn the Kd LED ON
      inputLEDs(false, false, false, true);
    }
  }
  // If on Manual Deflection mode
  else if (currentMode == 2) 
  {
    // If was not on Manual Deflection mode before
    if (mode != 2) 
    {
      // Transmit the instructions to activate a Manual Deflection mode on aircraft
      String cmd = "Manual Deflection";
  
      String response = transmit(rf69, rf69_manager, RADIO_RX_ADDRESS, cmd);
  
      if (response.indexOf("Manual Defl Activated!") != -1) 
      {
        // update the parameters
        currentPitchAngle = getNumberFromString(response, "Pitch Angle: ");
        deflAngle = getNumberFromString(response, "Defl Angle: ");

        // switch Controlled mode status LED OFF
        // switch Manual defl. mode status LED ON
        statusLEDs(false, true, false);

        setLcdData(deflAngle, currentPitchAngle, refPitchAngle, Kp, Ki, Kd);
      } 
    }

    // set the mode
    mode = currentMode;
    
    float potValPrevious = potVal;

    potVal = getCurrentPotValue();

    if (potVal > potValPrevious + potValTolerance || potVal < potValPrevious - potValTolerance)
    {
      // set the Elevator Deflection value and send it to the aircraft
      deflAngle = mapPotVal(potVal, "Deflection");

      String cmd = "Deflection: " + String(deflAngle);

      String response = transmit(rf69, rf69_manager, RADIO_RX_ADDRESS, cmd);
      
      if (response.indexOf("Deflection: ") != -1) 
      {
        // update the parameters
        currentPitchAngle = getNumberFromString(response, "Pitch Angle: ");

        // blink Manual Defl. mode status LED
        statusLEDBlink(MANUALDEFL_LED_PIN);

        setLcdData(deflAngle, currentPitchAngle, refPitchAngle, Kp, Ki, Kd);
      }
    }
  }
  // If on Neutral mode
  else if (currentMode == 1) 
  {
    // If was not on Neutral mode before
    if (mode != 1) 
    {
      // Transmit the instructions to activate a Neutral mode on aircraft
      String cmd = "Neutral";
      
      String response = transmit(rf69, rf69_manager, RADIO_RX_ADDRESS, cmd);
    
      if (response.indexOf("Neutral Mode Activated!") != -1) 
      {
        // update the parameters
        currentPitchAngle = getNumberFromString(response, "Pitch Angle: ");
        deflAngle = getNumberFromString(response, "Defl Angle: ");

        // switch Controlled mode status LED OFF
        // switch Manual defl. mode status LED OFF
        statusLEDs(false, false, false);

        setLcdData(deflAngle, currentPitchAngle, refPitchAngle, Kp, Ki, Kd);
      } 
    }
    
    // set the mode
    mode = currentMode;    
  }

  // if not on the Controlled mode, unset the Controlled mode inputs
  if (currentMode != 0)
  {
    controlledModeInput = -1;
    inputLEDs(false, false, false, false);
  }
}

int getCurrentMode() 
{
  int controlledMode = 0;
  int manualDeflMode = 0;
  int neutralMode = 0;

  controlledMode = digitalRead(CONTROLLED_BTN_PIN);
  manualDeflMode = digitalRead(MANUALDEFL_BTN_PIN);
  neutralMode = digitalRead(NEUTRAL_BTN_PIN);

  if (controlledMode == HIGH) return 0;
  if (manualDeflMode == HIGH) return 2;
  if (neutralMode == HIGH) return 1;
  
  return -1;
}

int getControlledModeInput()
{
  int pitchInput = 0;
  int PGainInput = 0;
  int IGainInput = 0;
  int DGainInput = 0;

  pitchInput = digitalRead(PITCH_INPUT_PIN);
  PGainInput = digitalRead(P_GAIN_INPUT_PIN);
  IGainInput = digitalRead(I_GAIN_INPUT_PIN);
  DGainInput = digitalRead(D_GAIN_INPUT_PIN);

  if (pitchInput == HIGH) return 0;
  if (PGainInput == HIGH) return 1;
  if (IGainInput == HIGH) return 2;
  if (DGainInput == HIGH) return 3;

  return -1;
}

float getCurrentPotValue() 
{
  float potVal = 0;

  potVal = analogRead(POT_CONTROL_PIN);
  //Serial.print("potVal: ");
  //Serial.println(potVal);

  return potVal;
}

void statusLEDs(bool controlledLedOn, bool manualDeflLedOn, bool radioLedOn) 
{
  if (controlledLedOn) 
  {
    Serial.println("Controlled mode LED on!");
    digitalWrite(CONTROLLED_LED_PIN, HIGH);
  }
  else 
  {
    Serial.println("Controlled mode LED off!");
    digitalWrite(CONTROLLED_LED_PIN, LOW);
  }

  if (manualDeflLedOn) 
  {
    Serial.println("Manual defl. mode LED on!");
    digitalWrite(MANUALDEFL_LED_PIN, HIGH);
  }
  else 
  {
    Serial.println("Manual defl. mode LED off!");
    digitalWrite(MANUALDEFL_LED_PIN, LOW);
  }

  if (radioLedOn) 
  {
    Serial.println("Radio status LED on!");
    digitalWrite(RADIO_LED_PIN, HIGH);
  }
  else 
  {
    Serial.println("Radio status LED off!");
    digitalWrite(RADIO_LED_PIN, LOW);
  }
}

void inputLEDs(bool pitchLedOn, bool KpLedOn, bool KiLedOn, bool KdLedOn)
{
  if (pitchLedOn) 
  {
    digitalWrite(PITCH_LED_PIN, HIGH);
  }
  else 
  {
    digitalWrite(PITCH_LED_PIN, LOW);
  }

  if (KpLedOn) 
  {
    digitalWrite(P_GAIN_LED_PIN, HIGH);
  }
  else 
  {
    digitalWrite(P_GAIN_LED_PIN, LOW);
  }

  if (KiLedOn) 
  {
    digitalWrite(I_GAIN_LED_PIN, HIGH);
  }
  else 
  {
    digitalWrite(I_GAIN_LED_PIN, LOW);
  }

  if (KdLedOn) 
  {
    digitalWrite(D_GAIN_LED_PIN, HIGH);
  }
  else 
  {
    digitalWrite(D_GAIN_LED_PIN, LOW);
  }
}

void statusLEDBlink(int LED_PIN)
{
  // if LED is radio status LED, light it up
  // otherwise, turn off for a bit
  if (LED_PIN == RADIO_LED_PIN)
  {
    digitalWrite(LED_PIN, HIGH);
    delay(50);
    digitalWrite(LED_PIN, LOW);
  }
  else
  {
    digitalWrite(LED_PIN, LOW);
    delay(50);
    digitalWrite(LED_PIN, HIGH);
  }
}

float mapPotVal(float potValue, String input)
{
  // set the mappings accrodingly
  // 0 angles is mapped at 90 angles of servo deflection
  if (input == "Pitch") return map(potValue, 0, 1023, 60, 125);
  if (input == "Deflection") return map(potValue, 0, 1023, 40, 130); 
  if (input == "PGain") return map(potValue, 0, 1023, 90, 100);
  if (input == "IGain") return map(potValue, 0, 1023, 90, 100);
  if (input == "DGain") return map(potValue, 0, 1023, 90, 100);

  // default mapping
  return map(potValue, 0, 1023, 0, 180);
}

float getNumberFromString(String str, String startPhrase) 
{
  //function to extract number from string
  int index = str.indexOf(startPhrase);

  int numberStartIndex = index + startPhrase.length();
  int numberEndIndex = str.length();
  float numberToReturn = 0;

  for (int i = numberStartIndex; i < str.length(); i++) 
  {
    // if not digit and not a minus sign and not dot
    if (!isdigit(str[i]) && str[i] != '-' && str[i] != '.')
    {
      numberEndIndex = i;
      break;
    }
  }

  String numberToReturnStr = str.substring(numberStartIndex, numberEndIndex);
  
  numberToReturn = atof(numberToReturnStr.c_str());
  
  return numberToReturn;
}
