#include <Zumo32U4.h>

Zumo32U4LCD lcd;
Zumo32U4ProximitySensors proxsensors;
Zumo32U4LineSensors lineSensors;
Zumo32U4Motors motors;
Zumo32U4ButtonA bA;

#define RIGHT 1
#define LEFT 0
#define NUM_SENSORS 5
uint16_t lineSensorValues[NUM_SENSORS];
uint8_t selectedSensorIndex = 0;


void setup() {
    lcd.clear();
    lcd.print(F("Let's get ready to rumble"));
    bA.waitForButton();
    delay(5000);
    ledRed(1);
    ledYellow(0);
    ledGreen(0);
    lcd.clear();
    proxsensors.initThreeSensors();
    lineSensors.initFiveSensors();
    loadCustomCharacters();
}
void loadCustomCharacters()
{
  static const char levels[] PROGMEM = {
    0, 0, 0, 0, 0, 0, 0, 63, 63, 63, 63, 63, 63, 63
  };
  lcd.loadCustomCharacter(levels + 0, 0);  // 1 bar
  lcd.loadCustomCharacter(levels + 1, 1);  // 2 bars
  lcd.loadCustomCharacter(levels + 2, 2);  // 3 bars
  lcd.loadCustomCharacter(levels + 3, 3);  // 4 bars
  lcd.loadCustomCharacter(levels + 4, 4);  // 5 bars
  lcd.loadCustomCharacter(levels + 5, 5);  // 6 bars
  lcd.loadCustomCharacter(levels + 6, 6);  // 7 bars
}

void printBar(uint8_t height)
{
  if (height > 8) { height = 8; }
  const char barChars[] = {' ', 0, 1, 2, 3, 4, 5, 6, 255};
  lcd.print(barChars[height]);
}

void printReadingsToLCD()
{
  // On the first line of the LCD, display the bar graph.
  lcd.gotoXY(0, 0);
  for (uint8_t i = 0; i < 5; i++)
  {
    uint8_t barHeight = map(lineSensorValues[i], 0, 2000, 0, 8);
    printBar(barHeight);
  }
// On the second line of the LCD, display one raw reading.
  lcd.gotoXY(0, 1);
  lcd.print(selectedSensorIndex);
  lcd.print(F(": "));
  lcd.print(lineSensorValues[selectedSensorIndex]);
  lcd.print(F("    "));
}

// Prints a line with all the sensor readings to the serial
// monitor.
void printReadingsToSerial()
{
  char buffer[80];
  sprintf(buffer, "%4d %4d %4d %4d %4d %c\n",
    lineSensorValues[0],
    lineSensorValues[1],
    lineSensorValues[2],
    lineSensorValues[3],
    lineSensorValues[4]
  );
  Serial.print(buffer);
}
void linesensors()
{
  if (lineSensorValues[selectedSensorIndex]>=1900){
    motors.setSpeeds(-200,-200);
    delay(1000);
    motors.setSpeeds(-100,100);
  }else {
    if (lineSensorValues[selectedSensorIndex]<=1900){
      motors.setSpeeds(200,200);
  }
}
}

void loop() {
    proxsensors.read();
    int left = proxsensors.countsLeftWithLeftLeds();
    int center_left = proxsensors.countsFrontWithLeftLeds();
    int center_right = proxsensors.countsFrontWithRightLeds();
    int right = proxsensors.countsRightWithRightLeds();

// prints the values of the prox sensors on the LCD screen
    lcd.gotoXY(0,1);
    lcd.print(left);
    lcd.print(" ");
    lcd.print(center_left);
    lcd.print(" ");
    lcd.print(center_right);
    lcd.print(" ");
    lcd.print(right);
    lcd.print(" ");

    static uint16_t lastSampleTime = 0;

    if ((uint16_t)(millis() - lastSampleTime) >= 100)
     {
        lastSampleTime = millis();

        // Read the line sensors.
        lineSensors.read(lineSensorValues);

        // Send the results to the LCD and to the serial monitor.
        printReadingsToLCD();
        printReadingsToSerial();
      }  
    linesensors();


    bool seenfront = center_left>=2 || center_right>=2; //this checks to see if the robot sees anyone in the front
    bool seenright = right>=2;
    bool seenleft = left>=2;
    
    bool attackfront = center_left==5 || center_right==5;
    
    if (seenfront) 
    {
        motors.setSpeeds(200,200);
    }
    else
        if (seenright)
        {
            motors.setSpeeds(200,0);
        }
        else
            if (seenleft)
            {
                motors.setSpeeds(0,200);
            }
            else
                motors.setSpeeds(-100,100);
    if (attackfront)
    {
        motors.setSpeeds(250,250);
    }




}
