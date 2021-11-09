#include <transceiver.h>
#include <LiquidCrystal_I2C.h>
#include <Wire.h>

// LEDs pins
#define MANUAL_LED_PIN    0
#define AUTO_LED_PIN      1

// Potentiometer pin
#define POT_CONTROL_PIN   21

// Buttons pins
//#define SWITCH_PIN        4
#define MANUAL_BTN_PIN    15
#define AUTO_BTN_PIN      16
#define NEUTRAL_BTN_PIN   17

// LCD pins
/*
#define LCD_RS_PIN        19      
#define LCD_EN_PIN        7
#define LCD_D4_PIN        14 
#define LCD_D5_PIN        15
#define LCD_D6_PIN        16
#define LCD_D7_PIN        17
*/

#define LCD_I2C_ADDR      0x3F

/*
#define RFM69_CS      4
#define RFM69_INT     3
#define RFM69_RST     2
#define LED           13
*/
#define RFM69_CS      10
#define RFM69_INT     4
#define RFM69_RST     9
#define LED           digitalPinToInterrupt(RFM69_INT)

#define RADIO_TX_ADDRESS     96
#define RADIO_RX_ADDRESS     69

//Adafruit_LiquidCrystal lcd(LCD_RS_PIN, LCD_EN_PIN, LCD_D4_PIN, LCD_D5_PIN, LCD_D6_PIN, LCD_D7_PIN);
LiquidCrystal_I2C lcd(LCD_I2C_ADDR, 20, 4);

// Singleton instance of the radio driver
RH_RF69 rf69(RFM69_CS, RFM69_INT);

// Class to manage message delivery and receipt
RHReliableDatagram rf69_manager(rf69, RADIO_TX_ADDRESS);

// 0 - manual
// 1 - neutral
// 2 - auto
int mode = -1;

int potValue = 0;
int pitchAngle = 0;

void setup() 
{
  Serial.begin(115200);
    
  lcd.init();                      // initialize the lcd 
  // Print a message to the LCD.
  lcd.backlight();
  //lcd.setCursor(3,0);
  //lcd.print("Hello, world!");
  
  setLcdData(potValue, pitchAngle);
  transceiverSetup(rf69, rf69_manager, RFM69_CS, RFM69_INT, RFM69_RST, LED);
  Serial.println("Radio init done");

  Serial.println("LCD init done");
  
  // display 0 degrees angle after setup
  
  pinMode(MANUAL_BTN_PIN, INPUT);
  pinMode(AUTO_BTN_PIN, INPUT);
  pinMode(NEUTRAL_BTN_PIN, INPUT);

  pinMode(MANUAL_LED_PIN, OUTPUT);
  pinMode(AUTO_LED_PIN, OUTPUT);
  
  statusLEDS(false, false);
}

void loop() 
{ 
  int switchPos = getSwitchPosition();
  int pitchAngle = 0;

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
  
      String response = transmit(rf69, rf69_manager, RADIO_RX_ADDRESS, cmd, LED);
      
      if (response.indexOf("Manual Mode Activated!") != -1) 
      {
        // update the pitch angle
        pitchAngle = getIntFromString(response, "Pitch Angle: ");

        // switch Manual status led ON
        // switch Auto status led OFF
        statusLEDS(true, false);
         
        Serial.println("Transmission successful!");
        Serial.print("Response: ");
        Serial.println(response);
      }
    }

    int currentPotValue = getCurrentPotValue();
    
    // if potentiometer value has changed
    if (currentPotValue != potValue) 
    {
      // TODO: also add some tolerance, pot value might fluctuate even when potentiometer is still??
      potValue = currentPotValue;

      // convert the pot value to string
      String potValueString = "PotValue: " + String(potValue);

      String response = transmit(rf69, rf69_manager, RADIO_RX_ADDRESS, potValueString, LED);

      if (response.indexOf("PotValue: ") != -1) 
      {
        // update the pitch angle
        pitchAngle = getIntFromString(response, "Pitch Angle: ");

        // blink Manual status led, 20ms delay
        digitalWrite(MANUAL_LED_PIN, LOW);
        delay(20);
        digitalWrite(MANUAL_LED_PIN, HIGH);

        setLcdData(potValue, pitchAngle);
        
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
  
      String response = transmit(rf69, rf69_manager, RADIO_RX_ADDRESS, cmd, LED);
  
      if (response.indexOf("Auto Mode Activated!") != -1) 
      {
        // update the pitch angle
        pitchAngle = getIntFromString(response, "Pitch Angle: ");

        setLcdData(potValue, pitchAngle);
        
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
      
      String response = transmit(rf69, rf69_manager, RADIO_RX_ADDRESS, cmd, LED);
    
      if (response.indexOf("Neutral Mode Activated!") != -1) 
      {
        // update the pitch angle
        pitchAngle = getIntFromString(response, "Pitch Angle: ");

        setLcdData(potValue, pitchAngle);
        
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

void setLcdData(int potValue, int pitchAngle) {
  // calculate the angle from the potentiometer voltage 
  // display the angle
  int deflAngle = map(potValue, 0, 1023, 0, 179);
  
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Defl angle ");
  lcd.print(deflAngle);
  lcd.setCursor(1, 0);
  lcd.print("Pitch angle ");
  lcd.print(pitchAngle);
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

int getIntFromString(String str, String startPhrase) 
{
  //function to extract number from string
  int index = str.indexOf(startPhrase);
  String substr = str.substring(index);

  int intStartIndex = -1;
  int intEndIndex = -1;
  int intToReturn = 0;

  for (int i = 0 ; i < substr.length(); i++) 
  {
    if (isdigit(substr[i]))
    {
      if (intStartIndex == -1) 
      {
        intStartIndex = i;
      }

      if ((i + 1) <= (substr.length() - 1))
      {
        if (!isdigit(substr[i+1]))
        {
          intEndIndex = i + 1;
        }
      }
      else 
      {
        intEndIndex = i;
      }
    }
  }

  substr = substr.substring(intStartIndex, intEndIndex);

  // convert the remaining text to an integer
  intToReturn = atoi(substr.c_str());
  
  return intToReturn;
}
