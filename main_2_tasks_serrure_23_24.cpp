#include <Arduino.h>
#include <schSTM32.h>

//#define project1  // LCD parallel Keypad I2C Relay
//#define project2 // LCD UART Keypad Analog Relay
//#define project3 // LCD I2C Keypad Direct  Relay
//#define project4 // OLED SPI Keypad Analog Relay
//#define project5 // LCD SPI Keypad direct Servo
//#define project6 // LCD parallel Keypad Analog Servo
#define project7 // LCD UART  Keypad I2C Servo
//#define project8 // OLED I2C Keypad Analog Servo
//#define project9

#if defined(project1)
#define DISP_LCD
//#define DISP_OLED
#define DISP_INTER_PAR
//#define DISP_INTER_I2C
//#define DISP_INTER_SPI
//#define DISP_INTER_UART
//#define KEYPAD_DIO
#define KEYPAD_DIO_I2C
//#define KEYPAD_ANALOG
#define O_RELAY
//#define O_SERVO
#endif

#if defined(project2)
#define DISP_LCD
//#define DISP_OLED
//#define DISP_INTER_PAR
//#define DISP_INTER_I2C
//#define DISP_INTER_SPI
#define DISP_INTER_UART
//#define KEYPAD_DIO
//#define KEYPAD_DIO_I2C
#define KEYPAD_ANALOG
#define O_RELAY
//#define O_SERVO
#endif

#if defined(project3)
#define DISP_LCD
//#define DISP_OLED
//#define DISP_INTER_PAR
#define DISP_INTER_I2C
//#define DISP_INTER_SPI
//#define DISP_INTER_UART
#define KEYPAD_DIO
//#define KEYPAD_DIO_I2C
//#define KEYPAD_ANALOG
#define O_RELAY
//#define O_SERVO
#endif

#if defined(project4)
//#define DISP_LCD
#define DISP_OLED
//#define DISP_INTER_PAR
//#define DISP_INTER_I2C
#define DISP_INTER_SPI
//#define DISP_INTER_UART
//#define KEYPAD_DIO
//#define KEYPAD_DIO_I2C
#define KEYPAD_ANALOG
#define O_RELAY
//#define O_SERVO
#endif
#if defined(project5)
#define DISP_LCD
//#define DISP_OLED
//#define DISP_INTER_PAR
//#define DISP_INTER_I2C
#define DISP_INTER_SPI
//#define DISP_INTER_UART
#define KEYPAD_DIO
//#define KEYPAD_DIO_I2C
//#define KEYPAD_ANALOG
//#define O_RELAY
#define O_SERVO
#endif

#if defined(project6)
#define DISP_LCD
//#define DISP_OLED
#define DISP_INTER_PAR
//#define DISP_INTER_I2C
//#define DISP_INTER_SPI
//#define DISP_INTER_UART
//#define KEYPAD_DIO
//#define KEYPAD_DIO_I2C
#define KEYPAD_ANALOG
//#define O_RELAY
#define O_SERVO
#endif

#if defined(project7)
#define DISP_LCD
//#define DISP_OLED
//#define DISP_INTER_PAR
//#define DISP_INTER_I2C
//#define DISP_INTER_SPI
#define DISP_INTER_UART
//#define KEYPAD_DIO
#define KEYPAD_DIO_I2C
//#define KEYPAD_ANALOG
//#define O_RELAY
#define O_SERVO
#endif

#if defined(project8)
//#define DISP_LCD
#define DISP_OLED
//#define DISP_INTER_PAR
#define DISP_INTER_I2C
//#define DISP_INTER_SPI
//#define DISP_INTER_UART
//#define KEYPAD_DIO
//#define KEYPAD_DIO_I2C
#define KEYPAD_ANALOG
//#define O_RELAY
#define O_SERVO
#endif

#if defined(project9)
//#define DISP_LCD
#define DISP_OLED
//#define DISP_INTER_PAR
#define DISP_INTER_I2C
//#define DISP_INTER_SPI
//#define DISP_INTER_UART
//#define KEYPAD_DIO
//#define KEYPAD_DIO_I2C
#define KEYPAD_ANALOG
#define O_RELAY
//#define O_SERVO
#endif

#if defined (DISP_LCD) && defined (DISP_INTER_PAR)
#include <LiquidCrystal.h>
//LCD interface
const int pinRS     = D2;
const int pinEnable = D3;
const int pinD4     = D4;
const int pinD5     = D5;
const int pinD6     = D6;
const int pinD7     = D7;

LiquidCrystal lcd(pinRS, pinEnable, pinD4, pinD5, pinD6, pinD7);
#endif

#if defined (DISP_LCD) && defined (DISP_INTER_UART)
HardwareSerial Serial1(USART1);// TX/RX PA9/PA10  D8/D2 only D8/UART2-TX is used
#define LCD_NHD_Serial Serial1 
#endif

#if defined (DISP_LCD) && defined (DISP_INTER_I2C)
#include <Wire.h>
#define LCD_I2C 0x28 // 0x50 shifted to right 
// SDA/SCL D14/D15 PB9/PB8
#endif

#if defined (DISP_LCD) && defined (DISP_INTER_SPI)
#include <SPI.h>
#define chipSelectPin D10
// SCK/MISO/MOSI D13/D12/D11
#endif

#if defined (DISP_OLED) && defined (DISP_INTER_SPI)
#include <SPI.h>
#include <U8x8lib.h>

U8X8_SSD1309_128X64_NONAME0_4W_HW_SPI u8x8(/* cs=*/ D10, /* dc=*/ D9, /* reset=*/ D8);

#endif

#if defined (DISP_OLED) && defined (DISP_INTER_I2C)
#include <wire.h>
#include <U8x8lib.h>

U8X8_SSD1306_128X64_NONAME_HW_I2C u8x8(/* reset=*/U8X8_PIN_NONE);

#endif
#if defined (KEYPAD_DIO) || defined (KEYPAD_DIO_I2C)
#include <Keypad.h>
char keys[4][4] = {
  {'1', '2', '3', 'F'},
  {'4', '5', '6', 'E'},
  {'7', '8', '9', 'D'},
  {'A', '0', 'B', 'C'}
};

#endif

#if defined (KEYPAD_DIO_I2C)

#include <Wire.h>
#include <Keypad_I2C.h>

#define I2CADDR 0x20


byte rowPins[4] = {3,2,1,0};      //connect to the row pinouts of the keypad
byte colPins[4] = {7,6,5,4}; //connect to the column pinouts of the keypad
Keypad_I2C keypad (makeKeymap (keys), rowPins, colPins, 4, 4, I2CADDR, PCF8574);
#endif

#if defined (KEYPAD_DIO)
//Clavier 4x4 pins:
const int ROW0 = D6;
const int ROW1 = D7;
const int ROW2 = D8;
const int ROW3 = D9;
const int COL0 = D2;
const int COL1 = D3;
const int COL2 = D4;
const int COL3 = D5;

const byte ROWS = 4; //four rows
const byte COLS = 4; //four columns

byte rowPins[ROWS] = {ROW0, ROW1, ROW2, ROW3};      //connect to the row pinouts of the keypad
byte colPins[COLS] = {COL0, COL1, COL2, COL3}; //connect to the column pinouts of the keypad
Keypad keypad = Keypad( makeKeymap(keys), rowPins, colPins, ROWS, COLS );

#endif


#if defined (KEYPAD_ANALOG)

// tested on board 7
uint16_t comp_value[17]={200,400,650,850,1100,1300,1500,1750,1950,2200,2400,2650,2950,3280,3600,3900,4095};

const char BUTTONS[16] = {'1','4','7','A','2','5','8','0','3','6','9','B','F','E','D','C'};

#define Keypad_analog_input A0

int Read_search(void)
{
  int adc_val;
  int index=16;
  
  adc_val = analogRead(Keypad_analog_input); // read again the analog input
      
  for (int i =0; i < 16 ; i++)
  {
    if( adc_val > comp_value[i]  && adc_val < comp_value[i+1] )
    { 
      index = i;  
      break;
    }   
  }
  return index;

}

char getKey(void)
{
  static uint8_t index0=16,index1=16;// static to be able non volatile from call to call
  uint8_t index2;
 
  index2 = Read_search();// in which intervall we are?

 if((index0 == 16) && (index1 < 16 ) && (index2 < 16) && (index1 == index2))// three samples the last two identical <16 and the first one is 16
   {
    index0=index1;//shifting index1 to index0
    index1=index2;//shiting index2 to index1
    return BUTTONS[index2];// button code returned
   }
   else // juste do shifting and return 0
   {
    index0=index1;
    index1=index2;
    return 0;
   }
}
#endif

#if defined (O_RELAY)
#define relay_pin PC0 //A5
#endif

#if defined (O_SERVO)
#include <Servo.h>
Servo servo_barriere ;
#endif

#if defined (DISP_LCD) && ( defined (DISP_INTER_UART)|| defined (DISP_INTER_I2C) || defined (DISP_INTER_SPI))
void LCD_write_byte(byte x)
{
#if defined (DISP_INTER_I2C)
Wire.beginTransmission(LCD_I2C);
Wire.write(x);  
Wire.write(0); 
Serial.println(Wire.endTransmission());
#elif defined (DISP_INTER_SPI)
digitalWrite(chipSelectPin, LOW);
SPI.transfer(x); 
// take the chip select high to de-select:
digitalWrite(chipSelectPin, HIGH);
delay(1);
#elif defined (DISP_INTER_UART)
LCD_NHD_Serial.write(x);
#endif 
}

void  LCD_NHD_Clear(void)
{
LCD_write_byte(0xFE);
LCD_write_byte(0x51);
delay(3);
}

void LCD_NHD_write_String(String x)
{
uint8_t length=x.length();
for (int i=0; i < length; i++)
LCD_write_byte(x[i]);
}

void LCD_NHD_setCursor(uint8_t col, uint8_t row)
{
  uint8_t offset = 0;
  if (row == 1 ) offset = 0x40; 
  LCD_write_byte(0xFE);
  LCD_write_byte(0x45);
  LCD_write_byte(col + offset);
}
#endif


#if defined (KEYPAD_ANALOG)
#define GetKey getKey
#elif defined (KEYPAD_DIO) || defined (KEYPAD_DIO_I2C)
#define GetKey keypad.getKey
#endif

#if defined (DISP_LCD) && defined (DISP_INTER_PAR) 
#define DISP_CLEAR lcd.clear
#define DISP_SETCURSOR lcd.setCursor
#define DISP_PRINT lcd.print
#endif

#if defined (DISP_LCD) && ( defined (DISP_INTER_I2C) || defined (DISP_INTER_SPI) || defined (DISP_INTER_UART))
#define DISP_CLEAR LCD_NHD_Clear
#define DISP_SETCURSOR LCD_NHD_setCursor
#define DISP_PRINT LCD_NHD_write_String
#endif

#if defined (DISP_OLED) && ( defined (DISP_INTER_I2C) || defined (DISP_INTER_SPI))
#define DISP_CLEAR u8x8.clear
#define DISP_SETCURSOR u8x8.setCursor
#define DISP_PRINT u8x8.print
#endif

 char Key;

 uint8_t Compteur_Etat =0;
 char code_readn[4];
 uint8_t counter=0;
 int j =0 ;
 String Txt ;
 char password[4]={'1','5','9','7'};
 #if defined (O_RELAY)
typedef enum {GET_CODE,CHECK_CODE,WAIT_500ms,WAIT_3S}ETAT;
String state_name[4]={"GET CODE","CHECK CODE","WAIT 500ms","WAIT 3S"};
#endif

#if defined (O_SERVO)
typedef enum {GET_CODE,CHECK_CODE,WAIT1_3s,WAIT2_3S, WAIT_KEY}ETAT;
String state_name[5]={"GET CODE","CHECK CODE","WAIT1 3S","WAIT2 3S","WAIT FOR KEY PRESS"};
#endif

 ETAT etat;

void Update_state(void)
{

switch(etat)
{
case GET_CODE : 
{
Key = GetKey();
if ( Key ) {  
switch (Key){
    case '0':
    case '1':
    case '2':
    case '3':
    case '4':
    case '5':
    case '6':
    case '7':
    case '8':
    case '9':
    code_readn[counter]=Key;
    Serial.println(Key); 
    DISP_PRINT("*");
    if (++counter == 4)
       {
       counter=0;
       etat=CHECK_CODE;
       Serial.print("Entring State:");
       Serial.println(state_name[etat]);
       }

}
}
break;
}
case CHECK_CODE : 
{
if(++Compteur_Etat == 2){// 2 *50 ms = 100 ms
    Compteur_Etat=0; // reset counter
    // checking code
  for (int i=0; i < 4; i++)
{
    if (code_readn[i]== password[i]) j++;
}
// clear LCD
DISP_CLEAR();
DISP_SETCURSOR(0, 0);
// test for correct code
if (j == 4) {
    //correct code
    Serial.println("Correct Code");
    DISP_PRINT("Correct Code");
    #if defined (O_RELAY)
    digitalWrite(relay_pin,HIGH);// Activate the relay
    etat=WAIT_500ms;
    #endif
    #if defined (O_SERVO)
    servo_barriere.write(90);// Activate the servo
    etat=WAIT1_3s;
    Serial.print("Entring State:");
    Serial.println(state_name[etat]);
    #endif
   
}else
{
//false code
Serial.println("Incorrect Code");
DISP_PRINT("Incorrect Code");
#if defined (O_RELAY)
etat=WAIT_3S;
#endif
#if defined (O_SERVO)
etat=WAIT2_3S;
Serial.print("Entring State:");
Serial.println(state_name[etat]);
#endif
}
j=0;//reset j used for counting the number of correct digit
}

break;
}
#if defined(O_SERVO)
case WAIT1_3s : 
{
 if(++Compteur_Etat == 60 )
{
  Compteur_Etat=0;
  Serial.println("waiting key press");
  DISP_CLEAR();
  DISP_SETCURSOR(0, 0);
  DISP_PRINT("Press any Key");
  DISP_SETCURSOR(0,1);
  DISP_PRINT("to close barrier");
  etat=WAIT_KEY;
  Serial.print("Entring State:");
  Serial.println(state_name[etat]);
}
break;
}
#endif 
#if defined (O_RELAY)
case WAIT_500ms:
{
 if(++Compteur_Etat == 10 ){// pulse duration = 10 *50 ms =500 ms
    Compteur_Etat=0;
    digitalWrite(relay_pin,LOW);// deactivate the relay
    etat=WAIT_3S;
    Serial.print("Entring State:");
    Serial.println(state_name[etat]);
}
break;
}
#endif
#if defined (O_RELAY)
case WAIT_3S : 
if(++Compteur_Etat == 60 ){// duration of 3 s before displaying the message for requesting new code
Compteur_Etat=0;
DISP_CLEAR();
DISP_SETCURSOR(0,0);
DISP_PRINT("Enter Code:");
DISP_SETCURSOR(0,1);
etat=GET_CODE;
Serial.print("Entring State:");
Serial.println(state_name[etat]);
}
#endif
#if defined (O_SERVO)
case WAIT_KEY:
{
 Key = GetKey();
if ( Key ) {
   servo_barriere.write(0);   // close barriar
   DISP_CLEAR();
   DISP_SETCURSOR(0,0);
   DISP_PRINT("Closing Barrier");
   etat=WAIT2_3S;
   Serial.print("Entring State:");
   Serial.println(state_name[etat]);
  }
break;
}

case WAIT2_3S:
{
 if(++Compteur_Etat == 60 ){// duration of 3 s before displaying the message for requesting new code
Compteur_Etat=0;
DISP_CLEAR();
DISP_SETCURSOR(0,0);
DISP_PRINT("Enter Code:");
DISP_SETCURSOR(0,1);
etat=GET_CODE;
Serial.print("Entring State:");
Serial.println(state_name[etat]);
}
break;
}
#endif  
}
}

void Update_LED(void)
{
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
}
void setup() {
// put your setup code here, to run once:
Serial.begin(115200);

#if defined (O_RELAY)
pinMode(relay_pin,OUTPUT);
digitalWrite(relay_pin,LOW);
Serial.println("Coded Lock Door");
#endif

#if defined (O_SERVO)
Serial.println("Coded Barrier");
servo_barriere.attach(PC7); 
servo_barriere.write(0);              	 //command to rotate the servo to the specified angle
delay(15);
#endif

#if defined (DISP_LCD) && defined (DISP_INTER_PAR)
lcd.begin(16, 2);
#endif

#if defined (DISP_LCD)
#if defined (DISP_INTER_UART)
LCD_NHD_Serial.begin(9600);
#elif defined (DISP_INTER_I2C)
Wire.begin();
#elif defined (DISP_INTER_SPI)
pinMode(chipSelectPin,OUTPUT);
SPI.beginTransaction(SPISettings(100000, MSBFIRST, SPI_MODE3));
#endif
delay(100);
#endif

#if defined (DISP_OLED) && (defined (DISP_INTER_SPI) || defined (DISP_INTER_I2C))
u8x8.begin();
u8x8.setFont(u8x8_font_amstrad_cpc_extended_r);
#endif
#if defined (KEYPAD_DIO_I2C)
Wire.begin (); // Call the connection Wire
keypad.begin (makeKeymap (keys));
#endif

#if defined (KEYPAD_ANALOG)
analogReadResolution(12);
#endif
etat=GET_CODE;
Serial.print("Entring State:");
Serial.println(state_name[GET_CODE]);
DISP_CLEAR();
DISP_SETCURSOR(0,0);
DISP_PRINT("Enter Code:");
DISP_SETCURSOR(0,1);

/**/
//Serial.println("Initialzing SCH...");
SCH_Init(20,HERTZ_FORMAT);// 50 ms  durée Tick 
SCH_Add_Task(Update_state,0,1);
#if !defined(DISP_INTER_SPI)
pinMode(LED_BUILTIN,OUTPUT);
SCH_Add_Task(Update_LED,1,10);
#endif
//Serial.println("Starting SCH...");
SCH_Start();
}

//loop function
void loop() {
SCH_Dispatch_Tasks();
}