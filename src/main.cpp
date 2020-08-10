/*
 * Dimmer LED CV7+2IN KNX 
 * 
 * Handle 7 LED channels
 * Handle 2 Input
 */

#include <Arduino.h>
#include <knx.h>

#define PIN_PROG_SWITCH   PA6
#define PIN_PROG_LED      PA7
#define PROG_TIMEOUT      ( 15 * 60 * 1000 )    // 15 mins
#define PIN_TPUART_RX     PB6   // stm32 knx uses Serial2 (pins 16,17)
#define PIN_TPUART_TX     PB7
#define PIN_TPUART_SAVE   PB3   // Unused
#define PIN_TPUART_RESET  PB4   // Unused

#define UPDOWN_INCREMENT    (5 * 10)
#define DPT_TO_DAC_FACTOR   (10)

#define PIN_IN1           PA5
#define PIN_IN2           PA4

#define MIN(X,Y)    ((X)<(Y)?(X):(Y))
#define MAX(X,Y)    ((X)>(Y)?(X):(Y))
#define DIFF(X,Y)    ((X)>(Y)?((X)-(Y)):((Y)-(X)))
#define INCREMENT(A,B,M)  (((M)-(B))<(A)?(M):(A)+(B))
#define DECREMENT(A,B,M)  (((M)+(B))>(A)?(M):(A)-(B))

static const uint16_t ledPins[] = { PA11, PA10, PA9, PA8, PA3, PA1, PA0};
enum { ledCount = sizeof(ledPins)/sizeof(ledPins[0]) };

static const uint16_t buttonPins[] = { PIN_IN1, PIN_IN2 };
enum { buttonCount = sizeof(buttonPins)/sizeof(buttonPins[0]) };

class NullStream : public Stream
{
public:
    virtual int available() { return 0; }
    virtual int read() { return -1; }
    virtual int peek() { return -1; }
    virtual void flush() {}
    virtual size_t write(uint8_t) { return 0; }
} nullDevice;

struct Button
{
    void init(int baseAddr, uint16_t baseGO, uint16_t pinNb)
    {
        m_params.value1bitOn = knx.paramByte(baseAddr);
        m_params.value1bitOff = knx.paramByte(baseAddr+1);
        m_params.value1byteOn = knx.paramInt(baseAddr+5);
        m_params.value1byteOff = knx.paramInt(baseAddr+9);
        m_GO.status = baseGO;
        knx.getGroupObject(m_GO.status).dataPointType(DPT_Switch);
        m_GO.status1byte = baseGO + 1;
        knx.getGroupObject(m_GO.status1byte).dataPointType(DPT_Scaling);
        m_GO.trigger = baseGO + 2;
        knx.getGroupObject(m_GO.trigger).dataPointType(DPT_Switch);
        m_GO.trigger1byte = baseGO + 3;
        knx.getGroupObject(m_GO.trigger1byte).dataPointType(DPT_Scaling);

        // Callbacks
        knx.getGroupObject(m_GO.status).callback([this](GroupObject& go) {
                m_status = (uint8_t)go.value() != m_params.value1bitOff;
            });
        knx.getGroupObject(m_GO.status1byte).callback([this](GroupObject& go) {
                m_status = (uint8_t)go.value() != m_params.value1byteOff;
            });

        m_pin = pinNb;
        pinMode(m_pin, INPUT);
        attachInterrupt(m_pin, [this](){
            m_status = true;
        }, RISING);
        attachInterrupt(m_pin, [this](){
            m_status = false;
        }, FALLING);
    }

    void loop()
    {
        if (m_lastEmittedStatus != m_status) {
            knx.getGroupObject(m_GO.status).value(m_status?m_params.value1bitOn:m_params.value1bitOff);
            knx.getGroupObject(m_GO.status1byte).value(m_status?m_params.value1byteOn:m_params.value1byteOff);
            if (m_status) {
                m_trigger = !m_trigger;
                knx.getGroupObject(m_GO.trigger).value(m_trigger);
                knx.getGroupObject(m_GO.trigger1byte).value(m_trigger?m_params.value1byteOn:m_params.value1byteOff);
            }
            m_lastEmittedStatus = m_status;
        }
    } 

  private:
    uint16_t m_pin;
    bool m_lastEmittedStatus = false;
    bool m_status = false;
    bool m_trigger = false;
    struct {
        uint8_t value1bitOn = true;
        uint8_t value1bitOff = false;
        uint8_t value1byteOn = 100;
        uint8_t value1byteOff = 0;
    } m_params;
    struct {
      uint16_t status;
      uint16_t status1byte;
      uint16_t trigger;
      uint16_t trigger1byte;
    } m_GO;
  public:
    enum { NBGO = sizeof(m_GO)/sizeof(uint16_t), SIZEPARAMS = sizeof(m_params) };
} button[buttonCount];

struct Dimmer
{
    void init(int baseAddr, uint16_t baseGO, int dimmerPin)
    {
        m_pin = dimmerPin;
        pinMode(m_pin, OUTPUT);
        m_params.minValue = knx.paramByte(baseAddr);
        m_params.maxValue = knx.paramByte(baseAddr+1);
        m_params.dimmingUpTime = knx.paramInt(baseAddr+2);
        m_params.dimmingDownTime = knx.paramInt(baseAddr+6);
        m_params.stairwayTimer1 = knx.paramInt(baseAddr+10);
        m_params.stairwayValue1 = knx.paramInt(baseAddr+14);
        m_params.stairwayTimer2 = knx.paramInt(baseAddr+18);
        m_params.mode = (Dimmer::PARAMS::Mode)knx.paramByte(baseAddr+22);
        m_GO.onoff = baseGO;
        knx.getGroupObject(m_GO.onoff).dataPointType(DPT_Switch);
        m_GO.status = baseGO+1;
        knx.getGroupObject(m_GO.status).dataPointType(DPT_Switch);
        m_GO.value = baseGO+2;
        knx.getGroupObject(m_GO.value).dataPointType(DPT_Scaling);
        m_GO.valueStatus = baseGO+3;
        knx.getGroupObject(m_GO.valueStatus).dataPointType(DPT_Scaling);
        m_GO.updown = baseGO+4;
        knx.getGroupObject(m_GO.updown).dataPointType(DPT_UpDown);
        m_GO.block = baseGO+5;
        knx.getGroupObject(m_GO.block).dataPointType(DPT_Switch);

        // Callbacks
        knx.getGroupObject(m_GO.onoff).callback([this](GroupObject& go) {
            if (knx.getGroupObject(m_GO.block).value())
                return;
            if (go.value()) {
                m_action = On;
                m_targetValue = m_params.maxValue;
            }
            else {
                m_action = Off;
                m_targetValue = 0;
            }
          });
        knx.getGroupObject(m_GO.value).callback([this](GroupObject& go) {
            if (knx.getGroupObject(m_GO.block).value())
                return;
            m_action = Dimming;
            m_targetValue = (int16_t)go.value() * DPT_TO_DAC_FACTOR;
          });
        knx.getGroupObject(m_GO.updown).callback([this](GroupObject& go) {
            if (knx.getGroupObject(m_GO.block).value())
                return;
            uint16_t val = go.value()?INCREMENT(m_targetValue, UPDOWN_INCREMENT, m_params.maxValue * DPT_TO_DAC_FACTOR):DECREMENT(m_targetValue, UPDOWN_INCREMENT, m_params.minValue * DPT_TO_DAC_FACTOR);
            if (m_targetValue != val) {
                m_action = Dimming;
                m_targetValue = val;
            }
          });
    }
  
    void loop(uint32_t delta)
    {
        bool dimmingFinished = false;
        uint16_t lastValue = m_currentValue;
        enum { None, Up, Down } move = None;
        switch (m_action) {
            case Dimmer::On:    move = Up; break;
            case Dimmer::Off:   move = Down; break;
            case Dimmer::Dimming: move = (m_currentValue < m_targetValue)?Up:Down; break;
            case Dimmer::StandBy: default: break;
        }
        if (m_params.mode != Dimmer::PARAMS::Simple) {
            if (m_targetValue / DPT_TO_DAC_FACTOR > m_params.minValue) {
                if (m_params.mode == Dimmer::PARAMS::Stairway1) {
                    if (m_timer > m_params.stairwayTimer1) {
                        move = (m_currentValue < m_params.minValue * DPT_TO_DAC_FACTOR)?Up:Down;
                        m_targetValue = m_params.minValue * DPT_TO_DAC_FACTOR;
                    }
                }
                else {
                    if (m_timer > m_params.stairwayTimer2) {
                        move = (m_currentValue < m_params.minValue * DPT_TO_DAC_FACTOR)?Up:Down;
                        m_targetValue = m_params.minValue * DPT_TO_DAC_FACTOR;
                    }
                    else if (m_timer > m_params.stairwayTimer1) {
                        move = (m_currentValue < m_params.stairwayValue1 * DPT_TO_DAC_FACTOR)?Up:Down;
                        m_targetValue = m_params.stairwayValue1 * DPT_TO_DAC_FACTOR;
                    }
                }
            }
        }
        switch (move) {
            case Up: {
                m_currentValue = INCREMENT(m_currentValue, m_params.dimmingDownTime * delta, m_targetValue);
                if (m_currentValue >= m_targetValue) {
                    dimmingFinished = true;
                }
            }; break;
            case Down: {
                m_currentValue = DECREMENT(m_currentValue, m_params.dimmingDownTime * delta, m_targetValue);
                if (m_currentValue <= m_targetValue) {
                    dimmingFinished = true;
                }              
            };
            default: break;
        }
        if (m_currentValue <= m_params.minValue * DPT_TO_DAC_FACTOR) {
            m_currentValue = m_params.minValue * DPT_TO_DAC_FACTOR;
            dimmingFinished = true;
        }
        else if (m_currentValue >= m_params.maxValue * DPT_TO_DAC_FACTOR) {
            m_currentValue = m_params.maxValue * DPT_TO_DAC_FACTOR;
            dimmingFinished = true;
        }
        m_timer += delta;
        // send to output
        analogWrite(m_pin, map(m_currentValue, 0, 100 * DPT_TO_DAC_FACTOR, 0, 65535));
        if (dimmingFinished || DIFF(m_currentValue, lastValue) > DPT_TO_DAC_FACTOR) {
            bool bOn = m_currentValue / DPT_TO_DAC_FACTOR > m_params.minValue;
            if (dimmingFinished) {
                m_action = Dimmer::StandBy;
                if (!bOn) m_timer = 0; 
                // send value to bus
                knx.getGroupObject(m_GO.valueStatus).value((uint8_t)(m_currentValue / DPT_TO_DAC_FACTOR));
                knx.getGroupObject(m_GO.status).value(bOn);
            }
            else {
                knx.getGroupObject(m_GO.valueStatus).valueNoSend((uint8_t)(m_currentValue / DPT_TO_DAC_FACTOR));
                knx.getGroupObject(m_GO.status).valueNoSend(bOn);
            }
        }
    }
  private:
    struct PARAMS {
      uint8_t minValue = 0;
      uint8_t maxValue = 100;
      uint32_t dimmingUpTime = 100;
      uint32_t dimmingDownTime = 100;
      enum Mode : uint8_t { Simple = 0, Stairway1, Stairway2 } mode;
      uint32_t stairwayTimer1 = 0;  // Wait while On
      uint32_t stairwayValue1 = 0;  // intermediate Value after timer1 
      uint32_t stairwayTimer2 = 0;  // Wait time with Intermediate Value before switching off 
    } m_params;
    // State
    enum { StandBy, On, Off, Dimming } m_action = Off;
    uint16_t m_currentValue = 0;
    uint16_t m_targetValue = 0;
    uint32_t m_timer = 0;
    uint16_t m_pin = 1;
    struct {
      uint16_t onoff = 1;
      uint16_t status = 1;
      uint16_t value = 1;
      uint16_t valueStatus = 1;
      uint16_t updown = 1;
      uint16_t block = 1;
    } m_GO;
  public:
    enum { NBGO = sizeof(m_GO)/sizeof(uint16_t), SIZEPARAMS = sizeof(m_params) };
} dimmer[ledCount];

struct LightChaser
{
    void init(int baseAddr, uint16_t baseGO)
    {
        m_params.lightCount = MIN(MAXLIGHTS, knx.paramByte(baseAddr));
        m_params.timer = knx.paramByte(baseAddr + 1);
        for (uint8_t i = 0; i < m_params.lightCount; ++i) {
            m_params.delayOn[i] = knx.paramInt(baseAddr + 5 + i * 4);
            if (m_maxDelayOn < m_params.delayOn[i]) m_maxDelayOn = m_params.delayOn[i];
        }
        for (uint8_t i = 0; i < m_params.lightCount; ++i) {
            m_params.delayOff[i] = knx.paramInt(baseAddr + 5 + (MAXLIGHTS + i) * 4);
            if (m_maxDelayOff < m_params.delayOff[i]) m_maxDelayOff = m_params.delayOff[i];
        }

        m_GO.OnOffUp = baseGO;
        knx.getGroupObject(m_GO.OnOffUp).dataPointType(DPT_Switch);
        m_GO.OnOffDown = baseGO + 1;
        knx.getGroupObject(m_GO.OnOffDown).dataPointType(DPT_Switch);
        m_GO.status = baseGO + 2;
        knx.getGroupObject(m_GO.status).dataPointType(DPT_Switch);
        for (uint8_t i = 0; i < m_params.lightCount; ++i) {
            m_GO.lightOnOff[i] = baseGO + 3 + i;
            knx.getGroupObject(m_GO.lightOnOff[i]).dataPointType(DPT_Switch);
        }
        m_GO.block = baseGO + 3 + MAXLIGHTS;
        knx.getGroupObject(m_GO.block).dataPointType(DPT_Switch);

        // Callbacks
        knx.getGroupObject(m_GO.OnOffUp).callback([this](GroupObject& go) {
            if (knx.getGroupObject(m_GO.block).value())
                return;
            m_action = go.value()?OnUp:OffUp;
          });
        knx.getGroupObject(m_GO.OnOffDown).callback([this](GroupObject& go) {
            if (knx.getGroupObject(m_GO.block).value())
                return;
            m_action = go.value()?OnDown:OffDown;
          });
    }
  
    void loop(uint32_t delta)
    {
        if (m_action == StandBy) return;
        if (m_action == OnUp) {
            if (m_timer < m_params.timer ) {
                m_timer = 0;
                m_action = OffUp;
            }
            else {
                if (m_timer == 0) {
                    knx.getGroupObject(m_GO.status).value(true);
                }
                for (uint8_t i = 0; i < m_params.lightCount; ++i) {
                    if (m_timer < m_params.delayOn[i]) {
                        knx.getGroupObject(m_GO.lightOnOff[i]).value(true);
                    }
                }
            }
        }
        if (m_action == OffUp) {
            bool bAllOff = true;
            for (uint8_t i = 0; i < m_params.lightCount; ++i) {
                if (m_timer < m_params.delayOff[i]) {
                    knx.getGroupObject(m_GO.lightOnOff[i]).value(false);
                }
                else bAllOff = false;
            }
            if (bAllOff) {
                knx.getGroupObject(m_GO.status).value(false);
                m_timer = 0;
                m_action = StandBy;
            }
        }
        if (m_action == OnDown) {
            if (m_timer < m_params.timer ) {
                m_timer = 0;
                m_action = OffDown;
            }
            else {
                if (m_timer == 0) {
                    knx.getGroupObject(m_GO.status).value(true);
                }
                for (uint8_t i = 0; i < m_params.lightCount; ++i) {
                    if (m_timer < m_maxDelayOn - m_params.delayOn[m_params.lightCount - 1 - i]) {
                        knx.getGroupObject(m_GO.lightOnOff[m_params.lightCount - 1 - i]).value(true);
                    }
                }
            }
        }
        if (m_action == OffDown) {
            bool bAllOff = true;
            for (uint8_t i = 0; i < m_params.lightCount; ++i) {
                if (m_timer < m_maxDelayOff - m_params.delayOff[m_params.lightCount - 1 - i]) {
                    knx.getGroupObject(m_GO.lightOnOff[m_params.lightCount - 1 - i]).value(false);
                }
                else bAllOff = false;
            }
            if (bAllOff) {
                knx.getGroupObject(m_GO.status).value(false);
                m_timer = 0;
                m_action = StandBy;
            }
        }
        m_timer += delta;
    }
  private:
    enum { MAXLIGHTS = 16 };
    struct PARAMS {
      uint8_t lightCount = 0;
      uint32_t timer;
      uint32_t delayOn[MAXLIGHTS];
      uint32_t delayOff[MAXLIGHTS];
    } m_params;
    uint32_t m_maxDelayOn = 0;
    uint32_t m_maxDelayOff = 0;
    enum { StandBy, OnUp, OffUp, OnDown, OffDown } m_action = StandBy;
    uint32_t m_timer = 0;
    struct {
      uint16_t OnOffUp;
      uint16_t OnOffDown;
      uint16_t status;
      uint16_t lightOnOff[MAXLIGHTS];
      uint16_t block;
    } m_GO;
  public:
    enum { NBGO = sizeof(m_GO)/sizeof(uint16_t), SIZEPARAMS = sizeof(m_params) };
} chaser;

static void blink(int nb) {
    for (int i = 0; i < nb; ++i) {
        digitalWrite(PIN_PROG_LED, 1);
        delay(250);
        digitalWrite(PIN_PROG_LED, 0);
        delay(250);
    }
    delay(1000);
}

void setup()
{
    ArduinoPlatform::SerialDebug = &nullDevice;
    static HardwareSerial serialTpuart(PIN_TPUART_TX, PIN_TPUART_RX);
    knx.platform().knxUart(&serialTpuart);
    knx.ledPin(PIN_PROG_LED);
    knx.ledPinActiveOn(HIGH);
    knx.buttonPin(PIN_PROG_SWITCH);
    knx.buttonPinInterruptOn(RISING);

    pinMode(PIN_PROG_LED, OUTPUT);
    blink(1);

    // Init device
    knx.version(1);                                         // PID_VERSION
    static const uint8_t orderNumber = 0;
    knx.orderNumber(&orderNumber);                          // PID_ORDER_INFO
    // knx.manufacturerId(0xfa);                               // PID_SERIAL_NUMBER (2 first bytes) - 0xfa for KNX Association
    knx.bauNumber(0x44494d52 /* = 'DIMR'*/);     // PID_SERIAL_NUMBER (4 last bytes)
    static const uint8_t hardwareType [] = { 0, 0, 0, 0, 0, 0 };
    knx.hardwareType(hardwareType);                         // PID_HARDWARE_TYPE
    knx.bau().deviceObject().induvidualAddress(1);

    // read adress table, association table, groupobject table and parameters from eeprom
    knx.readMemory();

    if (knx.configured()) {
        uint16_t offsetGO = 1; int offsetParam = 0;
        for (uint16_t i = 0; i < buttonCount; ++i, offsetGO += Button::NBGO, offsetParam += Button::SIZEPARAMS) {
            button[i].init(offsetParam, offsetGO, ledPins[i]);
        }
        for (uint16_t i = 0; i < ledCount; ++i, offsetGO += Dimmer::NBGO, offsetParam += Dimmer::SIZEPARAMS) {
            dimmer[i].init(offsetParam, offsetGO, ledPins[i]);
        }
        chaser.init(offsetParam, offsetGO);
    }

    // start the framework.
    knx.start();

    pinMode(PIN_PROG_LED, OUTPUT);
    blink(1);
}

void loop() 
{
    // don't delay here to much. Otherwise you might lose packages or mess up the timing with ETS
    knx.loop();

    // only run the application code if the device was configured with ETS
    if(knx.configured()) {
        static uint32_t lastTime = millis();
        uint32_t time = millis();
        const uint32_t delta = time - lastTime;
        if (delta > 10) {
            for (int i = 0; i < buttonCount; ++i) {
                button[i].loop();
            }
            for (int i = 0; i < ledCount; ++i) {
                dimmer[i].loop(delta);
            }
            chaser.loop(delta);
            lastTime = time;
        }
    }

    static uint32_t timerProgMode = 0;
    if (knx.progMode()) {
        if (timerProgMode == 0) {
            timerProgMode = millis();
        }
        else {
            if (millis() - timerProgMode > PROG_TIMEOUT) {
                knx.progMode(false);
                timerProgMode = 0;
            }
        }
    }
    else {
        timerProgMode = 0;
    }
}
