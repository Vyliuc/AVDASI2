#include <transceiver.h>
#include <LiquidCrystal_I2C.h>
#include <Wire.h>

// LEDs pins
#define MANUAL_LED_PIN    0
#define AUTO_LED_PIN      1

// Potentiometer pin
#define POT_CONTROL_PIN   21

// Buttons pins
#define MANUAL_BTN_PIN    15
#define AUTO_BTN_PIN      16
#define NEUTRAL_BTN_PIN   17

// LCD I2C channel
#define LCD_I2C_ADDR      0x3F

#define RADIO_TX_ADDRESS     96
#define RADIO_RX_ADDRESS     69

LiquidCrystal_I2C lcd(LCD_I2C_ADDR, 20, 4);

// Singleton instance of the radio driver
RH_RF69 rf69(RFM69_CS, RFM69_INT);

// Class to manage message delivery and receipt
RHReliableDatagram rf69_manager(rf69, RADIO_TX_ADDRESS);

// 0 - manual
// 1 - neutral
// 2 - auto
int mode = -1;

float deflAngle = 0;
float pitchAngle = 0;

void setLcdData(float deflAngle, float pitchAngle) {
  // display the angles
  lcd.clear();
  
  lcd.setCursor(0,1);
  lcd.print("Defl angle ");
  lcd.print(deflAngle);
  
  lcd.setCursor(0, 2); 
  lcd.print("Pitch angle "); 
  lcd.print(pitchAngle);
}

void setup() 
{
  Serial.begin(115200);
    
  lcd.init();                      // initialize the lcd 
  lcd.backlight();

  // display 0 degrees angles after setup
  setLcdData(deflAngle, pitchAngle);
  
  transceiverSetup(rf69, rf69_manager);
  
  pinMode(MANUAL_BTN_PIN, INPUT);
  pinMode(AUTO_BTN_PIN, INPUT);
  pinMode(NEUTRAL_BTN_PIN, INPUT);

  pinMode(MANUAL_LED_PIN, OUTPUT);
  pinMode(AUTO_LED_PIN, OUTPUT);
  
  statusLEDS(false, false);
}

void loop() 
{ 
  // Listen for pitch & deflection angles when the Auto mode is on
  String response_received = receive(rf69, rf69_manager, 0);

  if (response_received.indexOf("Pitch Angle:") != -1 && response_received.indexOf("Defl Angle:") != -1) 
  {
    Serial.println("GOT RESPONSE IN AUTO");
    Serial.println(response_received);

    // blink Auto status led, 20ms delay
    digitalWrite(AUTO_LED_PIN, LOW);
    delay(20);
    digitalWrite(AUTO_LED_PIN, HIGH);

    pitchAngle = getNumberFromString(response_received, "Pitch Angle: ");
    Serial.println(pitchAngle);
    deflAngle = getNumberFromString(response_received, "Defl Angle: ");
    Serial.println(deflAngle);

    setLcdData(deflAngle, pitchAngle);
  }

  // set the mode
  int switchPos = getSwitchPosition();

  if (switchPos == -1) 
  {
    switchPos = mode;
  }
  
  // If the switch is on Manual mode
  if (switchPos == 0)
  {
    // If the switch was NOT on Manual
    if (mode != 0) 
    {
      // Transmit the instructions to activate a Manual mode on aircraft
      String cmd = "Manual";
  
      String response = transmit(rf69, rf69_manager, RADIO_RX_ADDRESS, cmd);
      
      if (response.indexOf("Manual Mode Activated!") != -1) 
      {
        // update the pitch angle
        pitchAngle = getNumberFromString(response, "Pitch Angle: ");

        // switch Manual status led ON
        // switch Auto status led OFF
        statusLEDS(true, false);
         
        Serial.println("Transmission successful!");
        Serial.print("Response: ");
        Serial.println(response);
      }
    }

    float currentPotValue = getCurrentPotValue();
    float currentDeflAngle = potValToAngle(currentPotValue);
    
    // if potentiometer value has changed (angle changed)
    if (currentDeflAngle != deflAngle) 
    {
      // TODO: also add some tolerance, pot value might fluctuate even when potentiometer is still??
      deflAngle = currentDeflAngle;

      // convert the angle value to string
      String deflAngleString = "DeflAngle: " + String(deflAngle);

      String response = transmit(rf69, rf69_manager, RADIO_RX_ADDRESS, deflAngleString);

      if (response.indexOf("DeflAngle: ") != -1) 
      {
        // update the pitch angle
        pitchAngle = getNumberFromString(response, "Pitch Angle: ");

        // blink Manual status led, 20ms delay
        digitalWrite(MANUAL_LED_PIN, LOW);
        delay(20);
        digitalWrite(MANUAL_LED_PIN, HIGH);

        setLcdData(deflAngle, pitchAngle);
        
        Serial.println("Transmission successful!");
        Serial.print("Response: ");
        Serial.println(response);
      }  
    }

    mode = 0;
  }
  // If the switch is on Auto mode
  else if (switchPos == 2) 
  {
    // If the switch was NOT on Auto
    if (mode != 2) 
    {
      // Transmit the instructions to activate an Auto mode on aircraft
      String cmd = "Auto";
  
      String response = transmit(rf69, rf69_manager, RADIO_RX_ADDRESS, cmd);
  
      if (response.indexOf("Auto Mode Activated!") != -1) 
      {
        // update the pitch angle
        pitchAngle = getNumberFromString(response, "Pitch Angle: ");

        setLcdData(deflAngle, pitchAngle);
        
        // switch Auto status led ON
        // switch Manual status led OFF
        statusLEDS(false, true);
        
        Serial.println("Transmission successful!");
        Serial.print("Response: ");
        Serial.println(response);
      } 
    }

    mode = 2;
  }
  // If the switch is on Neutral
  else if (switchPos == 1) 
  {
    // If the switch was NOT on Neutral
    if (mode != 1) 
    {
      // Transmit the instructions to activate a Neutral mode on aircraft
      String cmd = "Neutral";
      
      String response = transmit(rf69, rf69_manager, RADIO_RX_ADDRESS, cmd);
    
      if (response.indexOf("Neutral Mode Activated!") != -1) 
      {
        // update the pitch angle
        pitchAngle = getNumberFromString(response, "Pitch Angle: ");

        setLcdData(deflAngle, pitchAngle);
        
        // switch Auto status led OFF
        // switch Manual status led OFF
        statusLEDS(false, false);
        
        Serial.println("Transmission successful!");
        Serial.print("Response: ");
        Serial.println(response);
      } 
    }
    
    mode = 1;    
  }
}

int getSwitchPosition() 
{
  int manualMode = 0;
  int autoMode = 0;
  int neutralMode = 0;

  manualMode = digitalRead(MANUAL_BTN_PIN);
  autoMode = digitalRead(AUTO_BTN_PIN);
  neutralMode = digitalRead(NEUTRAL_BTN_PIN);

  if (manualMode == HIGH) 
  {
    // switch to Manual
    //Serial.println("MANUAL btn is HIGH");
    return 0;
  }
  else if (autoMode == HIGH)
  {
    // switch to Auto
    //Serial.println("AUTO btn is HIGH");
    return 2;
  }
  else if (neutralMode == HIGH)
  { 
    // switch to Neutral
    //Serial.println("NEUTRAL btn is HIGH");
    return 1;
  }
  else 
  {
    return -1;
  }
}

int getCurrentPotValue() 
{
  int potVal = 0;

  potVal = analogRead(POT_CONTROL_PIN);
  Serial.print("potVal: ");
  Serial.println(potVal);

  // return the potentiometer value
  return potVal;
}

void statusLEDS(bool manualLedOn, bool autoLedOn) {
  if (manualLedOn) 
  {
    Serial.println("Manual LED on!");
    digitalWrite(MANUAL_LED_PIN, HIGH);
  }
  else 
  {
    Serial.println("Manual LED off!");
    digitalWrite(MANUAL_LED_PIN, LOW);
  }

  if (autoLedOn) 
  {
    Serial.println("Auto LED on!");
    digitalWrite(AUTO_LED_PIN, HIGH);
  }
  else 
  {
    Serial.println("Auto LED off!");
    digitalWrite(AUTO_LED_PIN, LOW);
  }
}

float potValToAngle(float potValue)
{
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
