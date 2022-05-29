
#include <Wire.h> // we are removing this because it is already added in liquid crystal library
#include <LiquidCrystal_I2C.h>
 
// Create the lcd object address 0x3F and 16 columns x 2 rows 
LiquidCrystal_I2C lcd (0x27, 16,2);  //
 
void  setup () {
   // Initialize the LCD connected 
  lcd.begin ();
  
  // Turn on the backlight on LCD. 
  lcd.backlight ();
  
  // print the Message on the LCD. 
  lcd.print ( "CIRCUITSCHOOLS." );
}
 
void  loop () {
    //Here cursor is placed on first position (col: 0) of the second line (row: 1) 
  lcd.setCursor (0, 1);
   // We write the number of seconds elapsed 
  lcd.print ( millis () / 1000);
  lcd.print ( " SECONDS" );
  delay (100);
}
