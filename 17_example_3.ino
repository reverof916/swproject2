#include <Servo.h>

// Arduino pin assignment

#define PIN_IR    0         // IR sensor at Pin A0
#define PIN_LED   9
#define PIN_SERVO 10

#define _DUTY_MIN 500  // servo full clock-wise position (0 degree)
#define _DUTY_NEU 1580  // servo neutral position (90 degree)
#define _DUTY_MAX 2400  // servo full counter-clockwise position (180 degree)

#define _DIST_MIN 100.0   // minimum distance 100mm
#define _DIST_MAX 250.0   // maximum distance 250mm

#define _EMA_ALPHA 0.2      // for EMA Filter

#define LOOP_INTERVAL 25   // Loop Interval (unit: msec)

Servo myservo;
unsigned long last_loop_time;   // unit: msec

float dist_prev = _DIST_MIN;
float dist_ema = _DIST_MIN;

void setup()
{
  pinMode(PIN_LED, OUTPUT);
  
  myservo.attach(PIN_SERVO); 
  myservo.writeMicroseconds(_DUTY_NEU);
  
  Serial.begin(1000000);    // 1,000,000 bps
}

void loop()
{
  unsigned long time_curr = millis();
  int duty;
  float a_value, dist_raw;

  // wait until next event time
  if (time_curr < (last_loop_time + LOOP_INTERVAL))
    return;
  last_loop_time += LOOP_INTERVAL;

  a_value = analogRead(PIN_IR);
  dist_raw = ((6762.0 / (a_value - 9.0)) - 4.0) * 10.0;

  if ((dist_raw == 0.0) || (dist_raw > _DIST_MAX)) {
      dist_raw = dist_prev; //_DIST_MAX + 10.0;    // Set Higher Value
      digitalWrite(PIN_LED, 1);       // LED OFF
  } else if (dist_raw < _DIST_MIN) {
      dist_raw = dist_prev; //_DIST_MIN - 10.0;    // Set Lower Value
      digitalWrite(PIN_LED, 1);       // LED OFF
  } else {    // In desired Range
      dist_prev = dist_raw;
      digitalWrite(PIN_LED, 0);       // LED ON      
  }
  // and turn on LED if the distance is in the range 

  dist_ema = (_EMA_ALPHA*dist_raw) + ((1-_EMA_ALPHA)*dist_ema);

  if (dist_ema <= 100.0) {
    duty = _DUTY_MIN;
  } else if (dist_ema >= 250.0) {
    duty = _DUTY_MAX;
  } else {
    duty = (1900/150*dist_ema) - (115000/150);
  }
      
  myservo.writeMicroseconds(duty);

  Serial.print("_DUTY_MIN:");  Serial.print(_DUTY_MIN);
  Serial.print("_DIST_MIN:");  Serial.print(_DIST_MIN);
  Serial.print(",IR:");        Serial.print(a_value);
  Serial.print(",dist_raw:");  Serial.print(dist_raw);
  Serial.print(",ema:");       Serial.print(dist_ema);
  Serial.print(",servo:");     Serial.print(duty);
  Serial.print(",_DIST_MAX:"); Serial.print(_DIST_MAX);
  Serial.print(",_DUTY_MAX:"); Serial.print(_DUTY_MAX);
  Serial.println("");
}
