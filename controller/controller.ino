#include <Adafruit_LiquidCrystal.h>
#include <transceiver.h>

// LEDs pins
#define MANUAL_LED_PIN    31
#define AUTO_LED_PIN      32

// Potentiometer pin
#define POT_CONTROL_PIN   41

// Buttons pins
//#define SWITCH_PIN        4
#define MANUAL_BTN_PIN    2
#define AUTO_BTN_PIN      3
#define NEUTRAL_BTN_PIN   5

// LCD pins
#define LCD_RS_PIN        6       
#define LCD_EN_PIN        8
#define LCD_WRITE1_PIN    7 
#define LCD_WRITE2_PIN    28
#define LCD_WRITE3_PIN    29
#define LCD_WRITE4_PIN    30

#define RADIO_TX_ADDRESS     96
#define RADIO_RX_ADDRESS     69

Adafruit_LiquidCrystal lcd(LCD_RS_PIN, LCD_EN_PIN, LCD_WRITE1_PIN, LCD_WRITE2_PIN, LCD_WRITE3_PIN, LCD_WRITE4_PIN);

// Singleton instance of the radio driver
RH_RF69 rf69(RFM69_CS, RFM69_INT);

// Class to manage message delivery and receipt
RHReliableDatagram rf69_manager(rf69, RADIO_TX_ADDRESS);

void setup() 
{
  transceiverSetup(rf69, rf69_manager);
  
  // display 0 degrees angle after setup
  displayDeflectionAngle(0);
}

// 0 - manual
// 1 - neutral
// 2 - auto
int mode = 1;

int potValue = 0;

void loop() 
{ 
  int switchPos = getSwitchPosition();
  int pitchAngle = 0;
  
  // If the switch is on Manual mode
  if (switchPos == 0)
  {
    // If the switch was NOT on Manual
    if (mode != 0) 
    {
      // Transmit the instructions to activate a Manual mode on aircraft
      String cmd = "Manual";
  
      String response = transmit(rf69, rf69_manager, RADIO_RX_ADDRESS, cmd);
  
      if (response.indexOf("Manual Mode Activated!")) 
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

      String response = transmit(rf69, rf69_manager, RADIO_RX_ADDRESS, potValueString);

      if (response.indexOf("PotValue: ")) 
      {
        // update the pitch angle
        pitchAngle = getIntFromString(response, "Pitch Angle: ");

        // blink Manual status led  1 time, 20 + 20ms delay
        Blink(MANUAL_LED_PIN, 20, 1);

        displayDeflectionAngle(potValue);
        
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
  
      if (response.indexOf("Auto Mode Activated!")) 
      {
        // update the pitch angle
        pitchAngle = getIntFromString(response, "Pitch Angle: ");

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
  
      if (response.indexOf("Neutral Mode Activated!")) 
      {
        // update the pitch angle
        pitchAngle = getIntFromString(response, "Pitch Angle: ");

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

  pinMode(MANUAL_BTN_PIN, INPUT);
  pinMode(AUTO_BTN_PIN, INPUT);
  pinMode(NEUTRAL_BTN_PIN, INPUT);

  manualMode = digitalRead(MANUAL_BTN_PIN);
  autoMode = digitalRead(AUTO_BTN_PIN);
  neutralMode = digitalRead(NEUTRAL_BTN_PIN);

  if (manualMode == HIGH) 
  {
    // switch to Manual
    Serial.println("MANUAL btn is HIGH");
    return 0;
  }
  else if (autoMode == HIGH)
  {
    // switch to Manual
    Serial.println("AUTO btn is HIGH");
    return 2;
  }
  else if (neutralMode == HIGH)
  { 
    // switch to Manual
    Serial.println("NEUTRAL btn is HIGH");
    return 1;
  }
  else 
  {
    return 1;
  }
}

int getCurrentPotValue() 
{
  int potVal = 0;

  Serial.begin(9600);

  potVal = analogRead(POT_CONTROL_PIN);
  Serial.print("potVal: ");
  Serial.println(potVal);

  // return the potentiometer value
  return potVal;
}

void displayDeflectionAngle(int potValue) {
  // calculate the angle from the potentiometer voltage 
  // display the angle
  Serial.begin(9600);
  Serial.println("I have reached the LCD!");
  int displayDefAngle = map(potValue, 0, 1023, 0, 179);
  int displayPitchAngle = 0;// TODO: pitch angle input

  lcd.begin(16, 2);
  lcd.print("Deflection angle:");
  lcd.setCursor(0, 1);
  lcd.print(displayDefAngle);
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