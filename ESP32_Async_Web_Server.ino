/*********
  Rui Santos
  Complete project details at https://randomnerdtutorials.com  
*********/

// Import required libraries
#include "WiFi.h"
#include "ESPAsyncWebServer.h"
#include "SPIFFS.h"

// Replace with your network credentials
const char* ssid = "4G-UFI-351D";
const char* password = "1234567890";

// Set LED GPIO
const int ledPin = 2;
// Stores LED state
String ledState;

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);
int Speed;
// Motor A
int motor1Pin1 = 27; 
int motor1Pin2 = 26; 
int enable1Pin = 14; 
int SliderValue;
// Motor B
int motor2Pin1 = 25; 
int motor2Pin2 = 33; 
int enable2Pin = 32; 
int valueString; 
// Setting PWM properties
const int freq = 30000;
const int pwmChannel1 = 0;
const int pwmChannel2 = 1;
const int resolution = 8; //resolution of pulse width mod 8bit = 0-255 that we can vary the value
//int dutyCycle = 200;
// Declare the pins to which the LEDs are connected 
int greenled = 4;
int redled = 15; 
int blueled = 18;
int yellowled = 21;
String greenstate = "off";// state of green LED
String redstate = "off";// state of red LED
String bluestate = "off";// state of green LED
String yellowstate = "off";// state of red LED
// Replaces placeholder with LED state value
String processor(const String& var){
  Serial.println(var);
  if(var == "STATE"){
    if(digitalRead(ledPin)){
      ledState = "ON";
    }
    else{
      ledState = "OFF";
    }
    Serial.print(ledState);
    return ledState;
  }
  return String();
}
 
void setup(){

  pinMode(greenled, OUTPUT);
  pinMode(redled, OUTPUT);
  pinMode(blueled, OUTPUT);
  pinMode(yellowled, OUTPUT);

  // sets the pins as outputs:
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(enable1Pin, OUTPUT);

  pinMode(motor2Pin1, OUTPUT);
  pinMode(motor2Pin2, OUTPUT);
  pinMode(enable2Pin, OUTPUT);
  
  // configure LED PWM functionalitites
  ledcSetup(pwmChannel1, freq, resolution);
  ledcSetup(pwmChannel2, freq, resolution);
  
  // attach the channel to the GPIO to be controlled
  ledcAttachPin(enable1Pin, pwmChannel1);
  ledcAttachPin(enable2Pin, pwmChannel2); // attach pwm to D25

  // Serial port for debugging purposes
  Serial.begin(115200);
  pinMode(ledPin, OUTPUT);

  // Initialize SPIFFS
  if(!SPIFFS.begin(true)){
    Serial.println("An Error has occurred while mounting SPIFFS");
    return;
  }

  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi..");
  }

  // Print ESP32 Local IP Address
  Serial.println(WiFi.localIP());
  // Route for root / web page
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/index.html", String(), false, processor);
  });
  
  // Route to load style.css file
  server.on("/style.css", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/style.css", "text/css");
  });

  // Route to set GPIO to HIGH
  server.on("/FORWARD", HTTP_GET, [](AsyncWebServerRequest *request){
    digitalWrite(ledPin, HIGH);
    Serial.println("Motor forward");
    MotorForward(Speed);   
    Serial.println("green on");
    //greenstate = "on";
    digitalWrite(greenled, HIGH);
    digitalWrite(yellowled, LOW);
    digitalWrite(redled, LOW);
    digitalWrite(blueled, LOW);
    request->send(SPIFFS, "/index.html", String(), false, processor);
  });

  // Route to set GPIO to HIGH
  server.on("/BACKWARD", HTTP_GET, [](AsyncWebServerRequest *request){
    digitalWrite(ledPin, HIGH);
    Serial.println("Motor Backward");
    MotorBackward(Speed);    
    //Serial.println("red on");
    greenstate = "on";
    digitalWrite(greenled, LOW);
    digitalWrite(yellowled, LOW);
    digitalWrite(redled, HIGH);
    digitalWrite(blueled, LOW);
    request->send(SPIFFS, "/index.html", String(), false, processor);
  });

  // Route to set GPIO to HIGH
  server.on("/RIGHT", HTTP_GET, [](AsyncWebServerRequest *request){
    digitalWrite(ledPin, HIGH);
    Serial.println("Motor Right");
    MotorRight(Speed);    
    Serial.println("yellow on");
    //greenstate = "on";
    digitalWrite(greenled, LOW);
    digitalWrite(yellowled, HIGH);
    digitalWrite(redled, LOW);
    digitalWrite(blueled, LOW);
    request->send(SPIFFS, "/index.html", String(), false, processor);
  });

  // Route to set GPIO to HIGH
  server.on("/LEFT", HTTP_GET, [](AsyncWebServerRequest *request){
    digitalWrite(ledPin, HIGH);
    Serial.println("Motor Left");
    MotorLeft(Speed);   
    //Serial.println("blue on");
    //greenstate = "on";
    digitalWrite(greenled, LOW);
    digitalWrite(yellowled, LOW);
    digitalWrite(redled, LOW);
    digitalWrite(blueled, HIGH);
    request->send(SPIFFS, "/index.html", String(), false, processor);
  });

    // Route to set GPIO to LOW
  server.on("/SLOW", HTTP_GET, [](AsyncWebServerRequest *request){
    digitalWrite(ledPin, LOW);
    //Serial.println("Motor Stop");
    Speed = 100;  
    digitalWrite(greenled, LOW);
    digitalWrite(yellowled, LOW);
    digitalWrite(redled, LOW);
    digitalWrite(blueled, LOW);
    request->send(SPIFFS, "/index.html", String(), false, processor);
  });

  server.on("/FAST", HTTP_GET, [](AsyncWebServerRequest *request){
    digitalWrite(ledPin, HIGH);
    Serial.println("Motor forward");
    digitalWrite(greenled, LOW);
    digitalWrite(yellowled, LOW);
    digitalWrite(redled, LOW);
    digitalWrite(blueled, LOW);
    Speed = 250;    
    request->send(SPIFFS, "/index.html", String(), false, processor);
  });
  
  server.on("/NORMAL", HTTP_GET, [](AsyncWebServerRequest *request){
    digitalWrite(ledPin, HIGH);
    Serial.println("Motor forward");
    digitalWrite(greenled, LOW);
    digitalWrite(yellowled, LOW);
    digitalWrite(redled, LOW);
    digitalWrite(blueled, LOW);
    Speed = 170;    
    request->send(SPIFFS, "/index.html", String(), false, processor);
  });
  
  // Route to set GPIO to LOW
  server.on("/STOP", HTTP_GET, [](AsyncWebServerRequest *request){
    digitalWrite(ledPin, LOW);
    Serial.println("Motor Stop");
    digitalWrite(greenled, LOW);
    digitalWrite(yellowled, LOW);
    digitalWrite(redled, LOW);
    digitalWrite(blueled, LOW);
    MotorStop();    
    request->send(SPIFFS, "/index.html", String(), false, processor);
  });

server.on("/Slider", HTTP_GET, [](AsyncWebServerRequest *request){
  digitalWrite(ledPin, LOW);
  digitalWrite(greenled, LOW);
  digitalWrite(yellowled, LOW);
  digitalWrite(redled, LOW);
  digitalWrite(blueled, LOW);
  if(request->hasParam("Speed")){
    String speedValue = request->getParam("Speed")->value();
    Speed = speedValue.toInt();
  }
  request->send(SPIFFS, "/index.html", String(), false, processor);
  Serial.println(Speed);
});

  // Start server
  
  server.begin();
}
 
void loop(){
}


void MotorForward(int Speed)
{
  ledcWrite(pwmChannel1, Speed);
  ledcWrite(pwmChannel2, Speed);  
  Serial.println("Moving Forward");
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, HIGH);
  digitalWrite(motor2Pin1, HIGH);  //swap motor wiring if spin in different direction!
  digitalWrite(motor2Pin2, LOW);  
  delay(200);
  
  
}

void MotorBackward(int Speed)
{
  ledcWrite(pwmChannel1, Speed);
  ledcWrite(pwmChannel2, Speed);
  Serial.println("Moving Backwards");
  digitalWrite(motor1Pin1, HIGH);
  digitalWrite(motor1Pin2, LOW);
  digitalWrite(motor2Pin1, LOW);
  digitalWrite(motor2Pin2, HIGH);  
  delay(200);
}

void MotorLeft(int Speed)
{
  ledcWrite(pwmChannel1, Speed);
  ledcWrite(pwmChannel2, Speed);
  Serial.println("Moving Left");
  digitalWrite(motor1Pin1,HIGH);
  digitalWrite(motor1Pin2,LOW);
  digitalWrite(motor2Pin1,LOW);
  digitalWrite(motor2Pin2,LOW);
  delay(200);
}

void MotorRight(int Speed)
{
  ledcWrite(pwmChannel1, Speed);
  ledcWrite(pwmChannel2, Speed);
  Serial.println("Moving Right");
  digitalWrite(motor1Pin1,LOW);
  digitalWrite(motor1Pin2,LOW);
  digitalWrite(motor2Pin1,HIGH);
  digitalWrite(motor2Pin2,LOW);
  delay(200);
}

void MotorStop()
{
  ledcWrite(pwmChannel1, Speed);
  ledcWrite(pwmChannel2, Speed);
  Serial.println("Motor stopped");
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, LOW);
  digitalWrite(motor2Pin1, LOW);
  digitalWrite(motor2Pin2, LOW);
  delay(200);
}
