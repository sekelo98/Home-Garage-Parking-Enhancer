#include <IRremote.h>
#include<esp_bt.h>
#include<LiquidCrystal_I2C.h>
#include <Wire.h>
#include <LcdBarGraphRobojax.h>
#include <NewPing.h>
#include "BluetoothSerial.h" 
#include <WiFi.h>


BluetoothSerial ESP_BT; 




#define TRIGGER_PIN  33  //  pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN     32  //  pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE 60 // Maximum distance we want to ping for (in centimetres). Maximum sensor distance is rated at 400-500cm.
#define TRIGGER_PIN2  17  // pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN2    16 //  pin tied to echo pin on the ultrasonic sensor.

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);// NewPing setup of pins and maximum distance.

NewPing sonar2(TRIGGER_PIN2, ECHO_PIN2, MAX_DISTANCE);

LiquidCrystal_I2C lcd(0x27, 16, 2);
LcdBarGraphRobojax lbg(&lcd, 16, 0, 0);  // -- creating 16 character long bargraph starting at char 0 of line 0 
LcdBarGraphRobojax rjx(&lcd, 16, 0, 0);


int ldr = 34; //for ldr reading
int RECV_PIN = 4;
int motor1Pin1 = 27; 
int motor1Pin2 = 26; 
int enable1Pin = 14;
int buzzer = 19;
int lights = 18;
int t = 0;
int red = 13;
int green = 25;
int blue = 23;


// Setting PWM properties
const int freq = 30000;
const int pwmChannel = 0;
const int resolution = 8;
int dutyCycle = 255;
int valu = 6;
int initial;
int finale;
int k;

IRrecv irrecv(RECV_PIN);

decode_results results;

void setup() {
  // sets the pins as outputs or input:
  ESP_BT.begin("Home Garage Parking Enhancer");        // Name of your Bluetooth interface -> will show up on your phone
  lcd.init();
  irrecv.enableIRIn();
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(enable1Pin, OUTPUT);
  pinMode(buzzer, OUTPUT);
  pinMode(15, OUTPUT);
  pinMode(red, OUTPUT);
  pinMode(green, OUTPUT);
  pinMode(blue, OUTPUT);
  pinMode(lights, OUTPUT);
  pinMode(ldr, INPUT);


  // configure LED PWM functionalitites
  ledcSetup(pwmChannel, freq, resolution);
  
  // attach the channel to the GPIO to be controlled
  ledcAttachPin(enable1Pin, pwmChannel);

  Serial.begin(115200);
  digitalWrite(lights, LOW);

}

int distanceCM = sonar.ping_cm();

void loop() {
int incoming; //Settings value from app

   incoming = ESP_BT.read();
   //Serial.print("incoming ");  
   //Serial.println(incoming);

  int touch = touchRead(T3);
    //Serial.print("TOUCH ");  
    //Serial.println(touch);

       if ( touch <= 10){
      digitalWrite(lights, LOW);
   }

  
if((incoming < 11) && (incoming > 1)){
  valu = incoming;
} 
    //Serial.println(valu);
 
    ledcWrite(pwmChannel, dutyCycle);
    if(irrecv.decode(&results)){

    switch(results.value)
    {
      case 0x40BF926D: // Serial.println("OPEN");// Green button  
                          digitalWrite(motor1Pin1, HIGH);
                          digitalWrite(motor1Pin2, LOW);
                          t = 1;
                          initial = distanceCM;
                          break;

      case 0x40BF12ED: // Serial.println("CLOSE");// Red button  
                         digitalWrite(motor1Pin1, LOW);
                         digitalWrite(motor1Pin2, HIGH);
                         t = 0;
                         finale = distanceCM;
                         break;
                         
    }
    irrecv.resume(); // Receive the next value
  }
int difference = finale - initial;
   if(difference < 0){
    k = 1;
   }
   if(difference > 0){
    k = 0;
   }


  if(t == 1){
    open();
    
  }
  
  if(t == 0){
    Close(k);
    
  }


}

void send_BT(int id, int value1) {       // function to write id and value to the bluetooth interface (and split value in MSB and LSB
  ESP_BT.write(128 + id);
  ESP_BT.write(floor(value1/128));       // MSB
  ESP_BT.write(value1%128);              // LSB
}


void open(){
   btStart(); //turn on bluetooh
   lcd.backlight();
   int val = analogRead(ldr);
   Serial.print("light intensity:  "); 
   Serial.println(val);
  
  
   // get distance in cm
   int distanceCM2 = sonar2.ping_cm();
   //Serial.println(distanceCM);
   //Serial.print("distance "); 
   //Serial.println(distanceCM2);
   rjx.clearLine(1);// clear line 1 to display fresh voltage value
               // -- draw bar graph from the analog value 
                          rjx.drawValue( distanceCM, MAX_DISTANCE);
                          lcd.setCursor (0,1); //
                          lcd.print("Distance:"); 
                          lcd.setCursor (10,1); //
                          lcd.print(distanceCM); // print
                          lcd.setCursor (14,1); //  
                          lcd.print("cm");   
                          delay(200);
   light(val);
   buzz(distanceCM);
   distance(distanceCM, distanceCM2 );
        
          
          
 
}


void light(int val){
    if(val >= 500){
            digitalWrite(lights, HIGH);
          }

          if(val <= 300){
            digitalWrite(lights, LOW);
          }
}


void buzz(int distanceCM){

   if(distanceCM <= valu){
              digitalWrite(buzzer, HIGH);         
   }    
  else{
    digitalWrite(buzzer, LOW);
   }
  
}


void distance(int distanceCM, int distanceCM2 ){
      if(distanceCM <= valu){
            digitalWrite(red, HIGH);
            digitalWrite(green, LOW);
            digitalWrite(blue, LOW);
            send_BT(3, distanceCM); 
             
          }
          if((distanceCM >= valu) && (distanceCM2 <= 25)){
            digitalWrite(red, LOW);
            digitalWrite(green, HIGH);
            digitalWrite(blue, LOW);
            send_BT(1, distanceCM); 
          
          }

          if((distanceCM >= 40) && (distanceCM2 >= 25)){
            digitalWrite(red, LOW);
            digitalWrite(green, HIGH);
            digitalWrite(blue, LOW);
            send_BT(1, distanceCM); 
          
          }
          
          if((distanceCM2 >=29 ) && (distanceCM >= valu)){
            digitalWrite(red, LOW);
            digitalWrite(green, LOW);
            digitalWrite(blue, HIGH);
           send_BT(2, distanceCM); 
             
          }    
}

void Close(int difference){
        delay(2000);
        lcd.noBacklight();
        digitalWrite(red, LOW);
        digitalWrite(green, LOW);
        digitalWrite(blue, LOW);
        digitalWrite(red, LOW);
        digitalWrite(buzzer, LOW);
        if(difference == 1){
            digitalWrite(lights, LOW);
        }
        btStop(); //diasble bluetooth
        
}
