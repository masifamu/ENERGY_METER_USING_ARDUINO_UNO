#include <EEPROM.h>

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <EEPROM.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     4 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define NUMFLAKES     10 // Number of snowflakes in the animation example

#define LOGO_HEIGHT   16
#define LOGO_WIDTH    16
static const unsigned char PROGMEM logo_bmp[] =
{ B00000000, B11000000,
  B00000001, B11000000,
  B00000001, B11000000,
  B00000011, B11100000,
  B11110011, B11100000,
  B11111110, B11111000,
  B01111110, B11111111,
  B00110011, B10011111,
  B00011111, B11111100,
  B00001101, B01110000,
  B00011011, B10100000,
  B00111111, B11100000,
  B00111111, B11110000,
  B01111100, B11110000,
  B01110000, B01110000,
  B00000000, B00110000 };

uint32_t timeCount=0;
uint8_t readValue = 0;
uint16_t eepromAdd = 0;
volatile uint16_t V=0,I=0;
float Vv=0.0,Iv=0.0,P=0.0,E=0.0;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);


  //setting the ADC
  InitADC();

  
  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3D for 128x64
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }
  // Clear the buffer
  display.clearDisplay();
  display.setTextSize(2);             // Normal 1:1 pixel scale
  display.setTextColor(WHITE);        // Draw white text
  display.setCursor(0,0);             // Start at top-left corner
  display.println(F(" Punchline"));
  display.println(F("  Energy"));
  display.println(F(" Pvt. Ltd."));
  display.display();
  delay(3000);


  
  //routine for reading the data
  if(Serial){
    Serial.println("To read write     :R");
    while(1){
      readValue = Serial.read();
      if(readValue == 'R'){//write R to read the data
        Serial.println("Reading stored data,Please wait........");
        while(eepromAdd++ <= 1023){Serial.print(eepromAdd); Serial.print(": "); Serial.println(EEPROM.read(eepromAdd));}
        Serial.println("Done reading");
        break;
      }
    }
  }
  setupTimerOneInterrupt();
  setupTimerTwoInterrupt();
}

void loop() {

  //Reading the voltage
  //V = V*(5.0f / 1024.0f) *11.0f;
  Vv = V*(5.0f / 1024.0f) *11.0f;
  Serial.println(V);

  //Reading the current
  //I = I*(5.0f / 1024.0f);
  Iv = I*(5.0f / 1024.0f);
  //calculating the instantaneous power
  P=Vv*Iv;

  
  
  
  //before writing the data to EEPROM make sure no serial comm is available.
  if(readValue != 'R'){
  // put your main code here, to run repeatedly:
  Serial.println("Writing data to EEPROM");
  }
  //updating the display every 2 seconds
  testdrawstyles(Vv,0,Iv,0,P,E,timeCount/60,0/*voltage,distance,current,maxCurrent,instPower,totalPower,totalT,ch*/);
}


void testdrawstyles(float v, float dist, float c, float maxC, float instP, float totalP, float totalT, unsigned char ch) {
  display.clearDisplay();
  display.setTextColor(BLACK, WHITE);        // Draw white text
  display.setCursor(0,0);             // Start at top-left corner

  display.setTextColor(WHITE); 
  display.setTextSize(2); 
  
  display.print(v,1);display.print(F("V "));
  display.println(int(dist/100.0f));//total distance , 100.0f is the display factor
  display.print(c,1);display.print(F("A "));//inst current
  display.print(int(maxC),1);display.println(F("A"));//max current
  display.print(int(instP));display.print(F("W "));//inst power
  display.print(int(totalT));display.println(F("M"));//total power
  display.print(totalP,1);display.print(F("WH "));//total power
  display.print(char(ch));//printing the charging and discharging flag
  display.display();
}

void displayString(String s1, String s2){
  display.clearDisplay();
  display.setTextColor(BLACK, WHITE);        // Draw white text
  display.setCursor(0,0);             // Start at top-left corner

  display.setTextColor(WHITE); 
  display.setTextSize(2); 
  display.println(s1);//total power
  display.println(s2);//total power
  display.println(F("Error"));//total power
  display.display();
}




void setupTimerOneInterrupt(){//at every second
  cli();
  //set timer1 interrupt at 1Hz
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0
  // set compare match register for 1hz increments
  OCR1A = 15624;// = (16*10^6) / (1*1024) - 1 (must be <65536)
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS10 and CS12 bits for 1024 prescaler
  TCCR1B |= (1 << CS12) | (1 << CS10);  
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);  
  sei();
}
ISR(TIMER1_COMPA_vect){//timer1 interrupt 1Hz toggles pin 13 (LED)
  timeCount++;
  E += P/60.0f/60.0f;
}
void setupTimerTwoInterrupt(){//at every 16ms
  //set timer2 interrupt at 8kHz
  cli();
  TCCR2A = 0;// set entire TCCR2A register to 0
  TCCR2B = 0;// same for TCCR2B
  TCNT2  = 0;//initialize counter value to 0
  // set compare match register for 8khz increments
  OCR2A = 249;// = (16*10^6) / (8000*8) - 1 (must be <256)
  // turn on CTC mode
  TCCR2A |= (1 << WGM21);
  // Set CS21 bit for 8 prescaler
  TCCR2B |= (1 << CS21)|(1<<CS22)|(1<<CS20);   
  // enable timer compare interrupt
  TIMSK2 |= (1 << OCIE2A);
  sei();
}
ISR(TIMER2_COMPA_vect){//timer1 interrupt 8kHz toggles pin 9
  static uint16_t Vavg,Iavg;
  static uint8_t countV,countI,select;
  if(select == 0){
    if(++countV <=16){
      Vavg += ReadADC(0);
    }else{
      V = Vavg>>4;
      countV = 0;
      Vavg=0;
    }
    select=1;
  }else{
    if(++countI <=16){
      Iavg += ReadADC(2);
    }else{
      I = Iavg>>4;
      countI = 0;
      Iavg=0;
    }
    select=0;
  }
}

/*.....................................................................................................*/
uint16_t readAnalogVoltage(uint8_t adcChannel){
  uint16_t value=0;
  for(int i=0;i<10;i++){
    if(i>=2) value=value+ReadADC(adcChannel); else ReadADC(adcChannel); //leaving the first two samples for avoiding any undesired value
  }
  return(uint16_t((value*1.0)/8.0));
}
/*.....................................................................................................*/
void InitADC()
{
 // Select Vref=AVcc
 ADMUX |= (1<<REFS0);
 //set prescaller to 128 and enable ADC 
 ADCSRA |= (1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0)|(1<<ADEN);    
}
/*.....................................................................................................*/
uint16_t ReadADC(uint8_t ADCchannel)
{
 //select ADC channel with safety mask
 ADMUX = (ADMUX & 0xF0) | (ADCchannel & 0x0F);
 //single conversion mode
 ADCSRA |= (1<<ADSC);
 // wait until ADC conversion is complete
 while( ADCSRA & (1<<ADSC) );
 return ADC;
}
