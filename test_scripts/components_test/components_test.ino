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

void setup() {
  // put your setup code here, to run once:
}

void loop() {
  // put your main code here, to run repeatedly:

  //test_modes(0);
  test_modes(1);
  //test_modes(2);
  
  //ctrlbtns("pitch");
  //ctrlbtns("kp");
  //ctrlbtns("ki");
  //ctrlbtns("kd");

  //ctrlbtns_led_status_test("pitch");
  //ctrlbtns_led_status_test("kp");
  //ctrlbtns_led_status_test("ki");
  //ctrlbtns_led_status_test("kd");

  //modeLED("ctrl")
  //modeLED("manual")
  //modeLED("radio")

  //mode_led_status_test("ctrl");
  //mode_led_status_test("manual");
  //mode_led_status_test("radio");

  //potentiometer_test();
}

void test_modes(int mode)
{
  if (mode == 0)
  {
    pinMode(CONTROLLED_BTN_PIN, INPUT);
    int state = digitalRead(CONTROLLED_BTN_PIN);

    if (state == HIGH) 
    {
      Serial.println("Controlled btn HIGH!");
      delay(15);
    }
  }
  else if (mode == 1)
  {
    pinMode(NEUTRAL_BTN_PIN, INPUT);
    int state = digitalRead(NEUTRAL_BTN_PIN);

    if (state == HIGH) 
    {
      Serial.println("Neutral btn HIGH!");
      delay(15);
    }
  }
  else if (mode == 2)
  {
    pinMode(MANUALDEFL_BTN_PIN, INPUT);
    int state = digitalRead(MANUALDEFL_BTN_PIN);

    if (state == HIGH) 
    {
      Serial.println("Manual defl btn HIGH!");
      delay(15);
    }
  }
}

void ctrlbtns (String ctrlcmd){
  if (ctrlcmd == "pitch")
  {
    pinMode(PITCH_INPUT_PIN, INPUT);
    int state = digitalRead(PITCH_INPUT_PIN);

    if (state == HIGH) 
    {
      Serial.println("Ref Pitch btn HIGH");
      delay(15);
    }
  }
  else if (ctrlcmd == "kp")
  {
    pinMode(P_GAIN_INPUT_PIN, INPUT);
    int state = digitalRead(P_GAIN_INPUT_PIN);

    if (state == HIGH) 
    {
      Serial.println("P Gain btn HIGH");
      delay(15);
    }
  }
  else if (ctrlcmd == "ki")
  {
    pinMode(I_GAIN_INPUT_PIN, INPUT);
    int state = digitalRead(I_GAIN_INPUT_PIN);

    if (state == HIGH) 
    {
      Serial.println("I Gain btn HIGH");
      delay(15);
    }
  }
  else if (ctrlcmd == "kd")
  {
    pinMode(D_GAIN_INPUT_PIN, INPUT);
    int state = digitalRead(D_GAIN_INPUT_PIN);

    if (state == HIGH) 
    {
      Serial.println("D Gain btn HIGH");
      delay(15);
    }
  }
}

void ctrlbtns_led_status_test(String ctrlcmd)
{
  if (ctrlcmd == "pitch")
  {
    Serial.println("Pitch LED should be on!");
    digitalWrite(PITCH_LED_PIN, HIGH); 
    delay(500);
    digitalWrite(PITCH_LED_PIN, LOW);
  }
  else if (ctrlcmd == "kp")
  {
    Serial.println("Kp LED should be on!");
    digitalWrite(P_GAIN_LED_PIN, HIGH); 
    delay(500);
    digitalWrite(P_GAIN_LED_PIN, LOW);
  }
  else if (ctrlcmd == "ki")
  {
    Serial.println("Ki LED should be on!");
    digitalWrite(I_GAIN_LED_PIN, HIGH); 
    delay(500);
    digitalWrite(I_GAIN_LED_PIN, LOW);
  }
  else if (ctrlcmd == "kd")
  {
    Serial.println("Kd LED should be on!");
    digitalWrite(D_GAIN_LED_PIN, HIGH); 
    delay(500);
    digitalWrite(D_GAIN_LED_PIN, LOW); 
  }
}

void mode_led_status_test(String modeLED)
{
  if (modeLED == "ctrl")
  {
    Serial.println("Ctrl LED should be on!");
    digitalWrite(CONTROLLED_LED_PIN, HIGH); 
    delay(500);
    digitalWrite(CONTROLLED_LED_PIN, LOW);
  }
  else if (modeLED == "manual")
  {
    Serial.println("Manual LED should be on!");
    digitalWrite(MANUALDEFL_LED_PIN, HIGH); 
    delay(500);
    digitalWrite(MANUALDEFL_LED_PIN, LOW);
  }
  else if (modeLED == "radio")
  {
    Serial.println("Radio LED should be on!");
    digitalWrite(I_GAIN_LED_PIN, HIGH); 
    delay(500);
    digitalWrite(I_GAIN_LED_PIN, LOW);
  }
}

void potentiometer_test()
{
  double potVal = analogRead(POT_CONTROL_PIN);
  Serial.print("Pot Value: ");
  Serial.println(potVal);
  delay(50);
}
