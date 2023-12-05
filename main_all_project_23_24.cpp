#include <Arduino.h>
//#define project1  // LCD parallel Keypad I2C
//#define project2 // LCD UART Keypad Analog
//#define project3 // LCD I2C Keypad Direct 
//#define project4 // OLED SPI Keypad Analog
//#define project5 // LCD SPI Keypad direct
//#define project6 // LCD parallel Keypad Analog
//#define project7 // LCD UART  Keypad I2C
#define project8 // OLED I2C Keypad Analog

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
HardwareSerial Serial1(USART1);// TX/RX PA9/PA10  D8/D2
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
const int ROW0 = D8;
const int ROW1 = D9;
const int ROW2 = D10;
const int ROW3 = D11;
const int COL0 = D4;
const int COL1 = D5;
const int COL2 = D6;
const int COL3 = D7;

const byte ROWS = 4; //four rows
const byte COLS = 4; //four columns

byte rowPins[ROWS] = {ROW0, ROW1, ROW2, ROW3};      //connect to the row pinouts of the keypad
byte colPins[COLS] = {COL0, COL1, COL2, COL3}; //connect to the column pinouts of the keypad
Keypad keypad = Keypad( makeKeymap(keys), rowPins, colPins, ROWS, COLS );

#endif


#if defined (KEYPAD_ANALOG)

uint16_t comp_value[17]={200,350,650,850,1100,1300,1500,1750,1950,2200,2400,2650,2950,3280,3600,3900,4095};

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
  #define NO_KEY 0
 int index, new_index;
 index = Read_search();
  if (index == 16) return 0;// non push button is detected
     else
     {
      delay(50);// wait for end of rebond
      new_index=Read_search();
      if (new_index == 16) return 0;// error rebond!!!
         else
        {
          if (new_index == index)// the first sample is confirmed by the second sample
          {
            while ( analogRead(Keypad_analog_input) > comp_value[0]); // read again the analog input is low that the first threshoold
            return BUTTONS[index];
          }    
          else
          return 0;
        }      
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
uint8_t length= x.length();
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

void setup() {

// put your setup code here, to run once:
Serial.begin(115200);
Serial.println("Test");

#if defined (O_RELAY)
pinMode(relay_pin,OUTPUT);
digitalWrite(relay_pin,HIGH);
#endif

#if defined (O_SERVO)
servo_barriere.attach(PC7); 
for(int angle = 0; angle <= 90; angle ++) 	 // command to move from 0 degrees to 180 degrees 
           {                                  
              servo_barriere.write(angle);              	 //command to rotate the servo to the specified angle
               delay(15);   
           }  
            delay(1000);
  
             for(int angle =90; angle>=0; angle--)     // command to move from 180 degrees to 0 degrees 
           {                                
             servo_barriere.write(angle);              //command to rotate the servo to the specified angle
             delay(15);                       
           } 
#endif

#if defined (DISP_LCD) && defined (DISP_INTER_PAR)
lcd.begin(16, 2);
lcd.setCursor(0, 0);
lcd.print("TP SEA/SEE");
lcd.setCursor(0,1);
lcd.print("TIC 3");
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

#if defined (DISP_LCD) && (defined (DISP_INTER_UART) || defined (DISP_INTER_I2C)|| defined (DISP_INTER_SPI))
LCD_NHD_Clear();
LCD_NHD_write_String("TP SEA/SEE");
LCD_NHD_setCursor(0,1);
LCD_NHD_write_String("TIC 3");
#endif

#if defined (DISP_OLED) && (defined (DISP_INTER_SPI) || defined (DISP_INTER_I2C))
u8x8.begin();
u8x8.setFont(u8x8_font_amstrad_cpc_extended_r);
u8x8.setCursor(0, 0);
u8x8.print("TP SEA/SEE");  
u8x8.setCursor(0,1);
u8x8.print("TIC3");  
#endif
#if defined (KEYPAD_DIO_I2C)
Wire.begin (); // Call the connection Wire
keypad.begin (makeKeymap (keys));
#endif

#if defined (KEYPAD_ANALOG)
analogReadResolution(12);
#endif
}

void loop() {

char Key;

#if defined (KEYPAD_ANALOG)
#define GetKey getKey
#elif defined (KEYPAD_DIO) || defined (KEYPAD_DIO_I2C)
#define GetKey keypad.getKey
#endif

Key = GetKey();
 if (Key != NO_KEY){
    Serial.println(Key); 
  }
}