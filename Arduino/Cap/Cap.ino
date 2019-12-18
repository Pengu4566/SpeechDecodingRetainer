#include <CapacitiveSensor.h>
#include <U8x8lib.h>
#include <HID.h>

U8X8_SH1106_128X64_NONAME_HW_I2C u8x8(/* reset=*/ U8X8_PIN_NONE);
const uint8_t colLow = 4;
const uint8_t colHigh = 13;
const uint8_t rowCups = 0;
const uint8_t rowState = 2;
const uint8_t rowTemp = 4;
const uint8_t rowTime = 6;

// 10M resistor between pins 5 & 10, pin 10 is sensor pin

CapacitiveSensor btn11 = CapacitiveSensor(5,A5);

int CAP_TRIGGER_VALUE = 3000;
char KEY_PRESSED = ' ';
char KEY_PRESSED_2 = ' ';
long FIRST = 0;
long SECOND = 0;
double TOTAL = 0;  
int FLAG = 0; 

void lcdBold(bool aVal) 
{
  if (aVal) 
  {
    u8x8.setFont(u8x8_font_victoriabold8_r); //BOLD
  } 
  else 
  {
    u8x8.setFont(u8x8_font_victoriamedium8_r); //NORMAL
  }
}

void lcdRun() 
{
  lcdBold(false);
}

void setup()                    
{
  Serial.begin(9600);
  FLAG = 0;
  u8x8.begin();
  lcdBold(true);
  u8x8.clear();
  u8x8.setCursor(0,rowCups);
  u8x8.print(F("Calc & Keyboard"));
  u8x8.setCursor(0,rowState);
  u8x8.print(__DATE__);
  u8x8.setCursor(0,rowTemp);
  u8x8.print(__TIME__);
  lcdRun();
}

void loop()                    
{
    long start = millis();
    long btnVal11 =  btn11.capacitiveSensor(30);

    
    Serial.println(btnVal11);

    

    KEY_PRESSED = GetInput(btnVal11, CAP_TRIGGER_VALUE);

    //calculator
    if(FLAG == 0){
      switch(KEY_PRESSED) 
      {
        case '0' ... '9': 
          FIRST = FIRST * 10 + (KEY_PRESSED - '0');
          
          u8x8.clear(); 
          u8x8.setCursor(0,rowCups); 
          u8x8.println(FIRST);
          //Serial.println(FIRST);
          break;
          
        case '+':
          u8x8.setCursor(0,2); 
          u8x8.println(F("+"));
                  
          FIRST = (TOTAL != 0 ? TOTAL : FIRST);
          SECOND = SecondNumber();
          TOTAL = FIRST + SECOND;
  
  
          FIRST = 0, SECOND = 0;
  
          //u8x8.clear(); 
          u8x8.setCursor(0,6); 
          u8x8.println(TOTAL);
  
          TOTAL = 0;
  
          break;
       }
       
       if(KEY_PRESSED == '*') 
       {
          FLAG = 1;
       }
    }
    //t9 keyboard
    else if (FLAG == 1)
    {
       Serial.println("T9 KEYBOARD");
       FLAG = 0;
    }

    KEY_PRESSED = ' ';
    TOTAL = 0;
    delay(100);
}

char GetInput(long btnVal11, int CAP_TRIGGER_VALUE)
{
  
  if (btnVal11 > CAP_TRIGGER_VALUE){KEY_PRESSED = '*';}

  //Serial.println(KEY_PRESSED);
  return KEY_PRESSED;
}

long SecondNumber()
{
  while(1)
  {
    delay(100);
    long btnVal11 =  btn11.capacitiveSensor(30);
    
    if (btnVal11 > CAP_TRIGGER_VALUE){KEY_PRESSED_2 = '*';}

    if(KEY_PRESSED_2 >= '0' && KEY_PRESSED_2 <= '9')
    {
      SECOND = SECOND * 10 + (KEY_PRESSED_2 - '0');
      //Serial.println(SECOND);
      u8x8.setCursor(0,4); 
      u8x8.println(SECOND);
    }
    
    if(KEY_PRESSED_2 == '=') 
    {
      KEY_PRESSED_2 = ' ';
      break;
    }
      
    KEY_PRESSED_2 = ' ';
  }
  
  return SECOND; 
}
