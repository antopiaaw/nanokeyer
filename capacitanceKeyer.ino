
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
  #define DAHPADDLE 3
  #define DITPADDLE 2
  #define buzzerPin  4
  #define BUTTON_PIN 5
  #define MAXWORD 1200
  #define DAHSPEED 3
  #define DITDAHSPACE 1
  #define LETTERSPACE 3
  #define WORDSPACE 10
  #define CAPACITANCEVALUE 2 //i set 2 set what value works for you
 struct TreeNode {
    char data;  // Morse code character
    struct TreeNode* left;// pointer to the leftnode
    struct TreeNode* right;//pointer to the right node
};

LiquidCrystal_I2C lcd(0x27,16,2);  // set the LCD address to 0x27 for a 16 chars and 2 line display

  uint8_t  wordsPerMinute;
  uint8_t  toggleWPM=12; //set default to 12wpm
  uint8_t  dit,dah,lastState;
  bool isDit=false,isDah=false;
  struct TreeNode *tempNode,*rootNode;
    char Letter;
   char charArray[16];

 struct TreeNode* createNode(char data) {
    struct TreeNode* newNode = (struct TreeNode*)malloc(sizeof(struct TreeNode));
    newNode->data = data;
    newNode->left = NULL;
    newNode->right = NULL;
    return newNode;
}


 /****************
   * Function to insert a node
   *
   * Parameter list;
   * TreeNode root base start
   * char data char to be used
   * const char* binary dot or dash
   * Return Value;
   * void
   * ************/
// Function to insert a Morse code character and its binary representation into the binary tree
void insert(struct TreeNode* root, char data, const char* binary) {
    struct TreeNode *current = root;
    for (int i = 0; binary[i] != '\0'; i++) {
        if (binary[i] == '.') {
            if (current->left == NULL) {
                current->left = createNode('\0');  // Use '\0' to indicate an internal node
            }
            current = current->left;
        } else if (binary[i] == '-') {
            if (current->right == NULL) {
                current->right = createNode('\0'); // Use '\0' to indicate an internal node
            }
            current = current->right;
        }
    }
    current->data = data;
}
 /****************
   * Function to find struct left most
   *
   * Parameter list;
   * TreeNode base root
   * char &letter letter to return
   * Return Value;
   * struct node ptr*
   * ************/
struct TreeNode* findLeft(struct TreeNode* root, char &Letter){
    struct TreeNode *current = root;
            current = current->left;
            if (&current->data != "\0") {
                Letter = current->data;  
             return current;
            }

} 
 /****************
   * Function to find struct right most
   *
   * Parameter list;
   *  * TreeNode base root
   * char &letter letter to return
   * Return Value;
   * struct node ptr*
   * ************/
   
struct TreeNode* findRight(struct TreeNode* root, char &Letter){
    struct TreeNode *current = root;
            current = current->right;
           if (&current->data !="\0") {
                Letter = current->data;
             return current;
           }

}
 /****************
   * Function to make a Dit
   *
   * Parameter list;
   * void
   * Return Value;
   * void
   * ************/

void doDit() {
  uint32_t Ctime;
  int i;
    Ctime = millis();
    while(Ctime+dit>=millis()){
     tone(buzzerPin,750); 
    }
    isDit=true;
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
    uint32_t Ctime;
    Ctime = millis();
    while(Ctime+(dit*DAHSPEED)>=millis()){
     tone(buzzerPin,750); 
    }
    isDah=true;
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


void addLetter(uint8_t letter){
  static uint8_t counter =0;
 char tempArray[16];

 
  if (counter< 15){
    charArray[counter]=letter;// add to list
    charArray[counter+1]="\0"; // add null terminated to make string
    counter++;
  }
  else
  {
     for (int i =0;i < 15;i++){
        tempArray[i]=charArray[i+1];//pop first list letter
      }
   tempArray[14]=letter; //append last letter in list
   tempArray[15]='\0';  //add null to make string
   
       for (int j =0;j < 16 ;j++){
         charArray[j]=tempArray[j]; //copy tempArray list string to charArray list
       }
    
  }
  lcd.setCursor(0,1); //set cursor to begining of lcd display
  lcd.printstr(charArray); //print null terminated string
}


void setup()
{
  Serial.begin(9600);
     pinMode( buzzerPin, OUTPUT);
     pinMode(BUTTON_PIN, INPUT_PULLUP);
     digitalWrite(BUTTON_PIN,HIGH);
     dit=MAXWORD / toggleWPM;//this is not
     dah = dit * DAHSPEED;//interesting this is global
     rootNode=createNode("\0");
     tempNode= rootNode;
     // initialize the lcd
     lcd.init();
     lcd.backlight();
     lcd.setCursor(0,0);
     lcd.print("2E0ADR");
     lcd.setCursor(7,0);
     lcd.print(toggleWPM);
     lcd.setCursor(11,0);
     lcd.print("WPM");
     //lcd.autoscroll();
     lcd.setCursor(0,1);

    // Insert Morse code characters and their binary representations
    insert(rootNode, 'E', ".");
    insert(rootNode, 'T', "-");
    insert(rootNode, 'A', ".-");
    insert(rootNode, 'N', "-.");
    insert(rootNode, 'M', "--");
    insert(rootNode, 'D', "-..");
    insert(rootNode, 'O', "---");
    insert(rootNode, 'I', "..");
    insert(rootNode, 'S', "...");
    insert(rootNode, 'B', "-...");
    insert(rootNode, 'U', "..-");
    insert(rootNode, 'F', "..-.");
    insert(rootNode, 'V', "...-");
    insert(rootNode, 'J', ".---");
    insert(rootNode, 'K', "-.-");
    insert(rootNode, 'H', "....");
    insert(rootNode, 'L', ".-..");
    insert(rootNode, 'X', "-..-");
    insert(rootNode, 'P', ".--.");
    insert(rootNode, 'R', ".-.");
    insert(rootNode, 'Q', "--.-");
    insert(rootNode, 'Z', "--..");
    insert(rootNode, 'C', "-.-.");
    insert(rootNode, 'G', "--.");
    insert(rootNode, 'W', ".--");
    insert(rootNode, 'Y', "-.--");
    insert(rootNode, '5', ".....");
    insert(rootNode, '1', ".----");
    insert(rootNode, '2', "..---");
    insert(rootNode, '3', "...--");
    insert(rootNode, '4', "....-");
    insert(rootNode, '6', "-....");
    insert(rootNode, '7', "--...");
    insert(rootNode, '8', "---..");
    insert(rootNode, '9', "----.");
    insert(rootNode, '0', "-----");
}


void loop()
{
static uint32_t  dahCtime,ditCtime;  
static uint32_t  lastCtime;
uint32_t         wordSpaceTime,temp;
static uint32_t  oldCtime,counter=0;
  
  uint8_t dahCycles=readCapacitivePin(DAHPADDLE);
    if(dahCycles > CAPACITANCEVALUE) {
      doDah();
      oldCtime=wordSpaceTime;
      wordSpaceTime=millis();//for word space 
      dahCtime=micros(); //for key up time
      tempNode=findRight(tempNode,Letter);
        if(tempNode== NULL)
             Letter=35;
    }
  uint8_t ditCycles = readCapacitivePin(DITPADDLE);
    if(ditCycles > CAPACITANCEVALUE){
      doDit();
      oldCtime=wordSpaceTime;
      wordSpaceTime=millis();//for word space 
      ditCtime=micros();
      tempNode=findLeft(tempNode,Letter);
         if(tempNode== NULL)
            Letter=35;

    }
  
    
    

  

int result= digitalRead(BUTTON_PIN);
      if(result == false && lastState == HIGH){ //button pressed and released
         toggleWPM ++;
         dit=MAXWORD / toggleWPM;//SET NEW VALUE
         lcd.setCursor(7,0);
         lcd.print("    ");
         lcd.setCursor(7,0);
         lcd.print(toggleWPM);
       }
         if (toggleWPM == 25) toggleWPM =1;//bounds of wpm

   // save the the last button state
  lastState=result;
  
  lastCtime=micros();
  if (( lastCtime-dahCtime > 20000)&&(lastCtime-dahCtime < 33400)){ //key up
        addLetter(Letter);
        //lcd.print(Letter);
        counter++;
        tempNode= rootNode;
  }
  
   if (( lastCtime-ditCtime > 20000 ) && ( lastCtime - ditCtime < 33000)){ //key up
     // Serial.println(lastCtime-ditCtime); 
       addLetter(Letter);
       //lcd.print(Letter);
       counter++;
       tempNode= rootNode;
   }
    temp=  wordSpaceTime - oldCtime ;
        if ((temp >=(long)dit*WORDSPACE)&&(isDit||isDah)){
              addLetter(32);// Add a space
         // lcd.print(" ");
             counter++;
         }
    isDit=false;
    isDah=false;

        }
