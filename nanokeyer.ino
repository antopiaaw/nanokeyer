
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
  #define DAHPADDLE 3
  #define DITPADDLE 2
  #define buzzerPin  4
  #define BUTTON_PIN 5
  #define MAXWORD 1200
  #define DAHSPEED 3
  #define CAPACITANCEVALUE 2 //i set 2 set what value works for you
  
LiquidCrystal_I2C lcd(0x27,16,2);  // set the LCD address to 0x27 for a 16 chars and 2 line display

  uint8_t  wordsPerMinute;
  uint8_t  toggleWPM=12; //set default to 12wpm
  uint8_t  dit,dah,lastState;


 /****************
   * Function to make a Dit
   * 
   * Parameter list;
   * void
   * Return Value;
   * void
   * ************/
  
void doDit() {
    tone(buzzerPin,750);
    delay(dit);
    noTone(buzzerPin);
    delay(dit);

 }
 
  /****************
   * Function to make a Dah
   * 
   * Parameter list;
   * void
   * Return Value;
   * void
   * ************/
void doDah() {

      tone(buzzerPin,750);
      delay(DAHSPEED * dit);
      noTone(buzzerPin);
      delay(dit);
}

// from https://playground.arduino.cc/Code/CapacitiveSensor/

    uint8_t readCapacitivePin(int pinToMeasure) {
      volatile uint8_t* port;
      volatile uint8_t* ddr;
      volatile uint8_t* pin;

      byte bitmask;
    port = portOutputRegister(digitalPinToPort(pinToMeasure));
    ddr = portModeRegister(digitalPinToPort(pinToMeasure));
    bitmask = digitalPinToBitMask(pinToMeasure);

    pin=portInputRegister(digitalPinToPort(pinToMeasure));

    *port &= ~(bitmask);
    *ddr |= bitmask;
    delay(1);

    noInterrupts();

    *ddr &= ~(bitmask);
    *port |= bitmask;

    uint8_t cycles = 17;
      if(* pin & bitmask) {cycles =0;}
    else if(* pin & bitmask) {cycles =1;}
    else if(* pin & bitmask) {cycles =2;}
    else if(* pin & bitmask) {cycles =3;}
    else if(* pin & bitmask) {cycles =4;}
    else if(* pin & bitmask) {cycles =5;}
    else if(* pin & bitmask) {cycles =6;}
    else if(* pin & bitmask) {cycles =7;}
    else if(* pin & bitmask) {cycles =8;}
    else if(* pin & bitmask) {cycles =9;}
    else if(* pin & bitmask) {cycles =10;}
    else if(* pin & bitmask) {cycles =11;}
    else if(* pin & bitmask) {cycles =12;}
    else if(* pin & bitmask) {cycles =13;}
    else if(* pin & bitmask) {cycles =14;}
    else if(* pin & bitmask) {cycles =15;}
    else if(* pin & bitmask) {cycles =16;}

    interrupts();

    /*
     * Discharge th pin again by setting it low and output
     * Its important to leave the pins low if you want to
     * be able to touch more than 1 sensor at a time - if
     * the sensor is left pulled high ,when you touch
     * two sensors,your body will transfer the charge between 
     * sensors
     * 
     */
     *port &= ~(bitmask);
     *ddr |= bitmask;

     return cycles;
}

void setup()
{ 
     pinMode( buzzerPin, OUTPUT);
     pinMode(BUTTON_PIN, INPUT_PULLUP);
     digitalWrite(BUTTON_PIN,HIGH);
     dit=MAXWORD / toggleWPM;//this is not
     dah = dit * DAHSPEED;//interesting this is global
 
               // initialize the lcd 
     lcd.init();
     // Print a message to the LCD.
     lcd.backlight();
     lcd.setCursor(0,0);
     lcd.print("Words Per Minute");
     lcd.setCursor(7,1);
     lcd.print(toggleWPM);
}


void loop()
{
  uint8_t dahCycles=readCapacitivePin(DAHPADDLE);
    if(dahCycles > CAPACITANCEVALUE) {
      doDah();
  
    } 
  uint8_t ditCycles = readCapacitivePin(DITPADDLE);
    if(ditCycles > CAPACITANCEVALUE){
      doDit();
       
    }

int result= digitalRead(BUTTON_PIN);
 if(result == false && lastState == HIGH){ //button pressed and released
         toggleWPM ++;
         dit=MAXWORD / toggleWPM;//SET NEW VALUE
         lcd.setCursor(7,1);
         lcd.print("    ");
         lcd.setCursor(7,1);
         lcd.print(toggleWPM);
 }
         if (toggleWPM == 25) toggleWPM =1;//bounds of wpm

   // save the the last state
  lastState=result;
}
