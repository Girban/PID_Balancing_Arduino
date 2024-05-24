#define SRVO_PIN 9

#define ECHO1_PIN 2
#define TRIG1_PIN 3

#define ECHO2_PIN 5
#define TRIG2_PIN 6

#define ACCURACY_FACTOR 3

#define K_P 0.9           // 0.9
#define K_I 0.4          // 0.4
#define K_D 0.2         // 0.2

#define MIN_ECHO 0
#define MAX_ECHO 1200

#define DELAY_PROXY 10
#define DELAY_SERVO 10

#define ECHO_2_MM 0.172

const long echoLength = MAX_ECHO - MIN_ECHO ;


float proportional=0, integral=0, derivative=0;
float dt, last_time=0, now;
float previous=0, output = 0;

#include <Servo.h>
Servo myservo;
int angle=90;

long duration1=0, duration2=0, duration=0;
long distance1=0, distance2=0, distance=0;
float delta=0;

long error=0 ;


void setup() {

  Serial.begin(9600);

  pinMode(LED_BUILTIN, OUTPUT);

  digitalWrite(LED_BUILTIN, LOW);

  delay(DELAY_SERVO);
  myservo.attach(SRVO_PIN);
  myservo.write(90);
  delay(DELAY_SERVO);


  delay(1000);

  pinMode(ECHO1_PIN, INPUT);
  pinMode(TRIG1_PIN, OUTPUT);

  pinMode(ECHO2_PIN, INPUT);
  pinMode(TRIG2_PIN, OUTPUT);


  delay(3000);
  digitalWrite(LED_BUILTIN, HIGH);

}

void loop() {
  delay(100);
  float time = micros()/ 1e6 ;

  proxyUltraSonic();

  error = (duration1 - duration2);  
  delta = PID( error );

  angle = map(delta, (-echoLength), (echoLength), 10, 170);

  myservo.write( int(170-angle) );
  delay(DELAY_SERVO);

  // // Serial.print("Time:");
  Serial.print( time );
  Serial.print(", ");

  // // Serial.print("Delta_mm:");
  Serial.print( distance1 - distance2 );
  Serial.print(", ");

  // // // Serial.print("PID_Output_deg:");
  // // Serial.print(angle);
  // // Serial.print(", ");

  // // // Serial.print("Diffrence:");
  // // Serial.print(abs( (distance1 - distance2)- delta*ECHO_2_MM ));
  // // Serial.print(", ");

  // // // Serial.print("Distance_1_mm:");
  // // Serial.print(distance1);
  // // Serial.print(", ");

  // // // Serial.print("Distance_2_mm:");
  // // Serial.print(distance2);
  // // Serial.print(", ");

  // // Serial.print("Center_mm:");
  Serial.println(0);

 
}

void proxyUltraSonic() {


  digitalWrite(TRIG1_PIN, LOW);
  delayMicroseconds(2);
  
  digitalWrite(TRIG1_PIN, HIGH);
  delayMicroseconds(10);

  digitalWrite(TRIG1_PIN, LOW);

  duration1 = pulseIn(ECHO1_PIN, HIGH);
  // duration1 = duration1 * 0.0344 / 2;
  delay(DELAY_PROXY);


  digitalWrite(TRIG2_PIN, LOW);
  delayMicroseconds(2);
  
  digitalWrite(TRIG2_PIN, HIGH);
  delayMicroseconds(10);

  digitalWrite(TRIG2_PIN, LOW);

  duration2 = pulseIn(ECHO2_PIN, HIGH);
  // duration2 = duration2 * 0.0344 / 2;
  delay(DELAY_PROXY);


  if ( duration1 > 1500 ) {
    if ( duration2 > echoLength/2 ) {
      duration1 = 0;
    } else {
      duration1 = echoLength/2;
    }

  } else if ( duration2 > 1500 ) {
    if ( duration1 > echoLength/2 ) {
      duration2 = 0;
    } else {
      duration2 = echoLength/2;
    }
  }

  distance1 = duration1 * ECHO_2_MM ;
  distance2 = duration2 * ECHO_2_MM ;


}

float PID(float error) {
  now = millis();
  dt = (now - last_time)/1000.00;
  last_time = now;

  proportional = error;
  integral += error * dt;
  derivative = (error - previous) / dt;
  previous = error;

  float output = (K_P * proportional) + (K_I * integral) + (K_D * derivative);

  if ( (K_I * integral) > echoLength ) {
    integral = echoLength/K_I;
  } else if ( (K_I * integral) < -echoLength ) {
    integral = -echoLength/K_I;
  }

  if (output > echoLength) {
    output = echoLength;
  } else if (output < -echoLength) {
    output = -echoLength;
  }

  return output;
}

