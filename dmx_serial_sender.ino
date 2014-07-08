//
// This version of dmx_serial_sender reads data from the
// DMX shield using custom code to read pins 3 and 4 from
// the DMX board, and it sends data using the standard
// Serial interface (pins 0 and 1).
//

#include <SoftwareSerial.h>

// Carefully tuned delays for my Ardiuno Uno.
#define DELAY_1_US \
        __asm__ __volatile__ ( \
                "nop" "\n\t"   \
                "nop" "\n\t"   \
                "nop" "\n\t"   \
                "nop" "\n\t"   \
                "nop" "\n\t"   \
                "nop" "\n\t"   \
                "nop" "\n\t"   \
                "nop" "\n\t"   \
                "nop" "\n\t"   \
                "nop" "\n\t"   \
                "nop" "\n\t"   \
                "nop" "\n\t"   \
                "nop" "\n\t"   \
                "nop" "\n\t"   \
                "nop" "\n\t"   \
                "nop" "\n\t"   \
        )

#define DELAY_2_US \
  DELAY_1_US; \
  DELAY_1_US

#define DELAY_AFTER_READ \
        __asm__ __volatile__ ( \
                "nop" "\n\t"   \
                "nop" "\n\t"   \
                "nop" "\n\t"   \
                "nop" "\n\t"   \
        )

#define DELAY_4_US_AFTER_READ \
  DELAY_AFTER_READ; \
  DELAY_1_US; \
  DELAY_1_US; \
  DELAY_1_US

const int ledPin = 13;
const int rxPin = 3;
static const int SERIAL_SPEED = 19200;

// Magic values at the beginning and end of the transmission,
// just so the caller knows that the entire transmission was
// successful.
static const unsigned char MAGIC_VALUE_START = 42;
static const unsigned char MAGIC_VALUE_END = 24;


void setup()
{
  pinMode ( ledPin, OUTPUT );
  pinMode ( rxPin, OUTPUT );
  
  Serial.begin(SERIAL_SPEED);
}

static const int DATA_SIZE = 32;
unsigned char data[DATA_SIZE] = {};
unsigned char dataToSend[DATA_SIZE] = {};
unsigned long lastReportedMs = 0;
unsigned long addToResetSignalTime = 0;

static const int dataBitDelayUs = 4;
static const int halfDataBitDelayUs = dataBitDelayUs / 2;

bool readPinUntilChange(unsigned char* pinValue, unsigned long* valueTime, unsigned long timeoutUs) {
  *pinValue = digitalRead(rxPin);
  unsigned long valueStart = micros();
  while (digitalRead(rxPin) == *pinValue) {
    if ((micros() - valueStart) > timeoutUs) {
      return false;
    }
  }
  *valueTime = micros() - valueStart;
  return true;
}

int ReadDmx() {
  int bytesRead = 0;
  if (digitalRead(rxPin) == HIGH) {
    memset(data, 0, sizeof(data));
    unsigned char* dataPtr = data;
    const unsigned char* dataEnd = dataPtr + DATA_SIZE;

    const uint8_t bit = digitalPinToBitMask(rxPin);
    const uint8_t port = digitalPinToPort(rxPin);

    unsigned char pinValue;
    unsigned long valueTime;
    if (readPinUntilChange(&pinValue, &valueTime, 2000) && (pinValue == HIGH) && (valueTime > 88)) {
      unsigned long resetStart = micros();
      while (((*portInputRegister(port) & bit) == 0) && (micros() - resetStart < 88)) {}
      if (micros() - resetStart > 88) {
        noInterrupts();
        while ((*portInputRegister(port) & bit) == 0) {}  // Read to the end of the reset.
        while ((*portInputRegister(port) & bit) != 0) {}  // Read the first mark.
        while ((*portInputRegister(port) & bit) == 0) {}  // First byte is always 0.  Wait for its stop bits.

        while(dataPtr < dataEnd) {
          // Remove the stop bits, which are variable length.  Note that idle is
          // also caught by this.  To ensure that this doesn't sit through an entire
          // idle signal, DATA_BYTES should be smaller than the number of transmitted
          // bytes.  What a hack.
          while ((*portInputRegister(port) & bit) != 0) {}
          *dataPtr = 0;
          //DELAY_2_US;
          *dataPtr |= ((*portInputRegister(port) & bit) ? 1 : 0) << 0;  // Start bit is always 0.
          DELAY_4_US_AFTER_READ;
          *dataPtr |= ((*portInputRegister(port) & bit) ? 1 : 0) << 0;  // Least significant bit.
          DELAY_4_US_AFTER_READ;
          *dataPtr |= ((*portInputRegister(port) & bit) ? 1 : 0) << 1;
          DELAY_4_US_AFTER_READ;
          *dataPtr |= ((*portInputRegister(port) & bit) ? 1 : 0) << 2;
          DELAY_4_US_AFTER_READ;
          *dataPtr |= ((*portInputRegister(port) & bit) ? 1 : 0) << 3;
          DELAY_4_US_AFTER_READ;
          *dataPtr |= ((*portInputRegister(port) & bit) ? 1 : 0) << 4;
          DELAY_4_US_AFTER_READ;
          *dataPtr |= ((*portInputRegister(port) & bit) ? 1 : 0) << 5;
          DELAY_4_US_AFTER_READ;
          *dataPtr |= ((*portInputRegister(port) & bit) ? 1 : 0) << 6;
          DELAY_4_US_AFTER_READ;
          *dataPtr |= ((*portInputRegister(port) & bit) ? 1 : 0) << 7;  // Most significant bit.
          DELAY_4_US_AFTER_READ;

          ++dataPtr;
        }

        interrupts();
        bytesRead = dataPtr - data;
      }
    }
  }
  
  return bytesRead;
}

unsigned long lastReadDmxMs = 0;

void loop()
{
  // Don't read the DMX signal too frequently, or we won't get interrupts for the
  // serial port.  Maybe.  This change was added when something else was causing
  // a problem, so it might not be necessary.  Seems like a good idea, though.
  if (millis() - lastReadDmxMs > 16) {
    int bytesRead = ReadDmx();

    if (bytesRead) {
      lastReadDmxMs = millis();
      memcpy(dataToSend, data, DATA_SIZE);
      if (data[0] > 127 )
        digitalWrite(ledPin, HIGH);
      else
        digitalWrite(ledPin, LOW);
    }
  }

  bool sendData = false;
  while (Serial.available()) {
    Serial.read();
    sendData = true;
  }
  if (sendData) {
    Serial.write(MAGIC_VALUE_START);
    Serial.write(dataToSend, DATA_SIZE);
    Serial.write(MAGIC_VALUE_END);
    // Wait for this to finish sending, so our next DMX read doesn't
    // mess with the signal.
    Serial.flush();
  }
}

