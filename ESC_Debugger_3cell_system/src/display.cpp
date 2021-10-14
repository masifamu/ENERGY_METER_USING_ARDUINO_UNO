#include <display.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// void testdrawstyles(float v, float dist, float c, float maxC, float instP, float totalP, float totalT,uint16_t rpm) {
//   display.clearDisplay();
//   display.setTextColor(BLACK, WHITE);        // Draw white text
//   display.setCursor(0,0);             // Start at top-left corner

//   display.setTextColor(WHITE); 
//   display.setTextSize(2); 
  
//   display.print(v,1);display.print(F("V "));
//   display.println(int(dist/100.0f));//total distance , 100.0f is the display factor
//   display.print(c,1);display.print(F("A "));//inst current
//   display.print(int(maxC),1);display.println(F("A"));//max current
//   display.print(int(instP));display.print(F("W "));//inst power
//   display.print(int(totalT));display.println(F("M"));//total power
//   display.print(totalP,1);display.print(F("WH "));//total power
//   display.print(RPM);//RPM
//   display.display();
// }