
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_I2CDevice.h>
#include <EEPROM.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     4 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);


#define vPerDiv     0.0048828125
#define wheelDia    0.65//meters
#define noOfHSCutsPerRev  266
//#define ENABLESERIALPLOTTER

int hall_signal=2;
uint32_t timeCount=0,prevTime=0;
uint8_t readValue = 0;
uint16_t eepromAdd = 0,eepromStoreAdd=0,sampleCount=0;
volatile uint16_t V=0,I=0,maxI=0,noOfHSCuts=0,RPM=0;
float Vv=0.0,Iv=0.0,maxIv=0.0,P=0.0,E=0.0,dist=0.0,battWireV=0.0;


void counter() {
  dist += 3.14f*wheelDia;
  noOfHSCuts++;
}

void testdrawstyles(float v, float dist, float c, float maxC, float instP, float totalP, float totalT,uint16_t rpm) {
  display.clearDisplay();

  display.setTextColor(WHITE); 

  display.setCursor(0,0);display.setTextSize(2);display.print(v,1);display.print(F("V"));
  display.setCursor(70,0);display.setTextSize(2);display.print(int(totalT));display.print(F("M"));//total power
  // display.setCursor(70,0);display.setTextSize(2);display.println(int(dist/100.0f));//total distance , 100.0f is the display factor
  display.setCursor(0,16);display.setTextSize(3);display.print(c,1);display.print(F("A"));//inst current
  display.setCursor(92,16);display.setTextSize(2);display.print(int(maxC),1);display.print(F("A"));//max current
  display.setCursor(0,40);display.setTextSize(3);display.print(int(instP));display.print(F("W"));//inst power
  
  display.setCursor(92,40);display.setTextSize(2);display.print(int(totalP));//total power
  display.setCursor(92,56);display.setTextSize(1);display.print(F("WH"));
  // display.print(RPM);//RPM
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
  display.display();
}

void displayCheckWireConnectionAndReboot(){
  display.clearDisplay();
  display.setTextColor(BLACK, WHITE);        // Draw white text
  display.setCursor(0,0);             // Start at top-left corner

  display.setTextColor(WHITE); 
  display.setTextSize(2); 
  display.println(F("Check Wire& Reboot"));//total power
  display.println(F("or Connectto PC"));//total power
  display.display();
}
void displayReadingDonePowerOff(){
  display.clearDisplay();
  display.setTextColor(BLACK, WHITE);        // Draw white text
  display.setCursor(0,0);             // Start at top-left corner

  display.setTextColor(WHITE); 
  display.setTextSize(2); 
  display.println(F("  Reading done,Turn"));//total power
  display.println(F("Power off!"));//total power
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
  if(timeCount%10 == 0){
    RPM = noOfHSCuts*6;
    noOfHSCuts = 0;
  } 
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

ISR(TIMER2_COMPA_vect){//timer1 interrupt at every 16msec
  static uint16_t Vavg,Iavg;
  static uint8_t countV,countI,select;
  uint16_t temp=0;
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
      temp = ReadADC(2);
      if(maxI < temp) maxI=temp;
      Iavg += temp;
    }else{
      I = Iavg>>4;
      countI = 0;
      Iavg=0;
    }
    select=0;
  }
  //calculating voltage
  Vv = V*vPerDiv*11.0f;

  //calculating current
  //I*vPerDiv=is coming out to be equal to 2.49 or 2.50 when there is no current flowing.
  //when charging subtract 2.51 then there will be max error of 0.02*15.151515=0.3030 amp
  //while charging reading less. Always add in the result to get the actual value.
  //While dischargin reading more, always subtract error from the result to get the actual value.
  Iv = (I*vPerDiv - 2.51);//default sensor value is 2.49V
  if(Iv < 0) Iv = 0;  //checking if the current is in negative direction.
  Iv = Iv*15.15151515f;//1000/66=15.151515

  //calculating power
  P = Vv*Iv;
  
  //calculating energy consumed
  E += P*0.00000444444f;//P*(16/1000)/60/60=P*0.00000444444
}

/*.....................................................................................................*/
uint16_t readAnalogVoltage(uint8_t adcChannel){
  uint16_t value=0;
  for(int i=0;i<10;i++){
    if(i>=2) value=value+ReadADC(adcChannel); else ReadADC(adcChannel); //leaving the first two samples for avoiding any undesired value
  }
  return(uint16_t((value*1.0)/8.0));
}




void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(hall_signal, INPUT_PULLUP);

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

  //deciding the reading factor
  battWireV = ReadADC(0)*vPerDiv*11.0f;
  //Serial.println(battWireV);
  //routine for reading the data
  //if batt wire voltage is higher than 10V meaning that the batt is connected to the device
  //if batt wire voltage is less thatn 10v meaning that the device is not connected to batt or being ready to read 
  //the stored eeprom data
  uint16_t storedSampCount=0,temporary=0;
  if(battWireV < 10.0f){
    displayCheckWireConnectionAndReboot();
#ifndef ENABLESERIALPLOTTER
    Serial.println("To read stored data, write :R");
#endif
    while(1){
      if(Serial.available()){
        readValue = Serial.read();
        if(readValue == 'R'){//write R to read the data
#ifdef ENABLESERIALPLOTTER
          Serial.print("RPM,InstPow,Current_");
          Serial.print(EEPROM.read(1022));Serial.print("V_");
          Serial.print(EEPROM.read(1016)*255+EEPROM.read(1017));Serial.print("WH_");
          Serial.print(EEPROM.read(1018)*255+EEPROM.read(1019));Serial.print("M_");
          Serial.print(EEPROM.read(1020)*255+EEPROM.read(1021));Serial.println("m");
#endif
#ifndef ENABLESERIALPLOTTER
          Serial.print("Voltage: ");Serial.print(EEPROM.read(1022));Serial.print("V, ");
          Serial.print("Energy: ");Serial.print(EEPROM.read(1016)*255+EEPROM.read(1017));Serial.println("WH");
          Serial.print("Time: ");Serial.print(EEPROM.read(1018)*255+EEPROM.read(1019));Serial.print("M, ");
          Serial.print("Distance: ");Serial.print(EEPROM.read(1020)*255+EEPROM.read(1021));Serial.println("m");
#endif
          //Serial.println(".....................................................");
          storedSampCount=EEPROM.read(0)*5;//each sample is of 5 bytes
          eepromAdd=0;//starting address where the RPM,P,I is stored
          while(++eepromAdd <= storedSampCount){
            
            temporary = EEPROM.read(eepromAdd)*255;
            temporary += EEPROM.read(++eepromAdd);
            Serial.print(temporary);

            temporary = EEPROM.read(++eepromAdd)*255;
            temporary += EEPROM.read(++eepromAdd);
            Serial.print(",");Serial.print(temporary);

            Serial.print(",");temporary = EEPROM.read(++eepromAdd);
            Serial.println(temporary);
          }
#ifndef ENABLESERIALPLOTTER
          Serial.println("Done reading");
#endif
          displayReadingDonePowerOff();
        }
      }
    }
  }
  attachInterrupt(digitalPinToInterrupt(hall_signal), counter, FALLING);
  setupTimerOneInterrupt();
  setupTimerTwoInterrupt();
}

void loop() {
  //calculating the max current.
  maxIv = (maxI*vPerDiv - 2.51);//default sensor value is 2.49V
  if(maxIv < 0) maxIv = 0;
  maxIv = maxIv*15.15151515f;//1000/66=15.151515
  //Serial.print("MaxI: ");Serial.println(maxIv);
  
  //before writing the data to EEPROM make sure no serial comm is available.
  //not equal to R means eeprom is not read before because batt voltage was more than 10V
  //here Vv is less than 10V means Wire was previously connected but now it's disconnected.
  if(readValue != 'R' && Vv < 10.0f && timeCount >=10){
    //Now we can store the data.
    displayString("  Storing","   data");
    delay(2000);
    //here i am storing only E,time,dist parameters
    //store the E in two bytes (q,r)
    EEPROM.write(1016,int(E)/255);
    EEPROM.write(1017,int(E)%255);
    //store the total time in two bytes (q,r)
    EEPROM.write(1018,int(timeCount/60)/255);
    EEPROM.write(1019,int(timeCount/60)%255);
    //store the distance in two bytes(q,r)
    EEPROM.write(1020,int(dist)/255);
    EEPROM.write(1021,int(dist)%255);

    //Write storing data to eeprom commands here
    displayString("  Stored     data","   Turn   Power off!");
    while(1);
  }
  
  //store current,P,RPM dynamically every 10 seconds
  //5 bytes will be consumed every 10 seconds.
  //1 byte every 2 sec, 1016byte in 1016*2/60=33.867min
  //if(timeCount%10 == 0 && timeCount != prevTime){
  if(timeCount != prevTime){//to store at every seconds
    if(eepromStoreAdd < 1016){
      //storing the number of data samples stored in EEPROM, it will help in reading the data
      EEPROM.write(0,++sampleCount);
      //storing the data RPM
      EEPROM.write(++eepromStoreAdd,int(RPM)/255);
      EEPROM.write(++eepromStoreAdd,int(RPM)%255);
      //storing the data inst Power
      EEPROM.write(++eepromStoreAdd,int(P)/255);
      EEPROM.write(++eepromStoreAdd,int(P)%255);
      //string the current
      EEPROM.write(++eepromStoreAdd,int(Iv));
      prevTime = timeCount;
    }
  }
  //storing voltage at 20second
  if(timeCount==20) EEPROM.write(1022,int(Vv));
  //Printing every parameters
  //Serial.print("Voltage:");Serial.println(Vv);
  //Serial.print("Distance:");Serial.println(dist);
  //Serial.print("Current:");Serial.println(Iv);
  //Serial.print("MaxI:");Serial.println(maxIv);
  //Serial.print("Power:");Serial.println(P);
  //Serial.print("Energy:");Serial.println(E);
  //Serial.print("Time:");Serial.println(timeCount/60);
  //Serial.print("RPM:");Serial.println(RPM);

  //updating the display here
  testdrawstyles(Vv,dist,Iv,maxIv,P,E,timeCount/60,RPM/*voltage,distance,current,maxCurrent,instPower,totalPower,totalT*/);
}