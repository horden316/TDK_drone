#include <IRremote.h>
#include <APA102.h>
int RECV_PIN = 2;
IRrecv irrecv(RECV_PIN);
decode_results results;
const int ledPinR =  5;
const int ledPinG =  4;
const uint8_t dataPin = 11;
const uint8_t clockPin = 12;
APA102<dataPin, clockPin> ledStrip;
int pin6 = 6;
int pin7 = 7;

// Set the number of LEDs to control.
const uint16_t ledCount = 72;

// We define "power" in this sketch to be the product of the
// 8-bit color channel value and the 5-bit brightness register.
// The maximum possible power is 255 * 31 (7905).
const uint16_t maxPower = 50 * 31;

// The power we want to use on the first LED is 1, which
// corresponds to the dimmest possible white.
const uint16_t minPower = 1;

// Calculate what the ratio between the powers of consecutive
// LEDs needs to be in order to reach the max power on the last
// LED of the strip.
const float multiplier = pow(maxPower / minPower, 1.0 / (ledCount - 1));


void setup() {
  Serial.begin(9600);
  Serial.println("Enabling IRin");
  irrecv.enableIRIn(); // 開始接收訊號！
  Serial.println("Enabled IRin");
  pinMode(ledPinR, OUTPUT);
  pinMode(ledPinG, OUTPUT);
//  pinMode(6, OUTPUT);
  pinMode(pin6, OUTPUT);
  pinMode(pin7, OUTPUT);

}
void sendRed(uint16_t power)
{
  // Choose the lowest possible 5-bit brightness that will work.
  uint8_t brightness5Bit = 1;
  while (brightness5Bit * 255 < power && brightness5Bit < 31)
  {
    brightness5Bit++;
  }

  // Uncomment this line to simulate an LED strip that does not
  // have the extra 5-bit brightness register.  You will notice
  // that roughly the first third of the LED strip turns off
  // because the brightness8Bit equals zero.
  //brightness = 31;

  // Set brightness8Bit to be power divided by brightness5Bit,
  // rounded to the nearest whole number.
  uint8_t brightness8Bit = (power + (brightness5Bit / 2)) / brightness5Bit;

  // Send the white color to the LED strip.  At this point,
  // brightness8Bit multiplied by brightness5Bit should be
  // approximately equal to power.
  ledStrip.sendColor(brightness8Bit, 0, 0, brightness5Bit);
}

void sendGreen(uint16_t power)
{

  uint8_t brightness5Bit = 1;
  while (brightness5Bit * 255 < power && brightness5Bit < 31)
  {
    brightness5Bit++;
  }

  uint8_t brightness8Bit = (power + (brightness5Bit / 2)) / brightness5Bit;

  ledStrip.sendColor(0, brightness8Bit , 0, brightness5Bit);
}

void loop() {
  digitalWrite(6, HIGH);
  ledStrip.startFrame();
  float power = maxPower;
  uint16_t i = 0;
  if (irrecv.decode(&results)) {
    Serial.println(results.value); //接收訊號，以16進位型式輸出到監控視窗

    if (results.value == 16633903) { //如果按下遙控器的特定鍵，就顯示訊息！
      digitalWrite(ledPinR, LOW);
      digitalWrite(ledPinG, HIGH);
      digitalWrite(pin6, HIGH);
      digitalWrite(pin7, LOW);
      for (i = 0; i < ledCount; i++)
      {
        sendRed(power);
      }

    }
    else if (results.value == 16609423) {
      digitalWrite(ledPinG, LOW);
      digitalWrite(ledPinR, HIGH);
      digitalWrite(pin7, HIGH);
      digitalWrite(pin6, LOW);
      for (i = 0; i < ledCount; i++)
      {
        sendGreen(power);

      }

    }
    irrecv.resume(); // 接著接收下一個訊號

  }
  delay(100);



}
