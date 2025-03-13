#include <Arduino.h>
#include "Protocol.h"

#define RX_PWR_PIN PIN_A5
#define TX_PWR_PIN PIN_A4

#define RX_DATA_PIN 2
#define TX_DATA_PIN 3
#define SWITCH_PWR_DELAY 50
#define MAX_DATA_SIZE 32
#define MAX_RETRY 3
#define RETRY_DELAY 100
#define RETRY_TIMEOUT 600

auto mRCSwitch = RCSwitch();
MSG mMsg;

inline void data2bin(char *bin, unsigned long data, const unsigned int bitLength)
{
    if (bitLength > MAX_DATA_SIZE)
        return;
    for (unsigned int i = 0; i < bitLength; ++i)
    {
        bin[bitLength - 1 - i] = (data & 1) ? '1' : '0';
        data >>= 1;
    }
    bin[bitLength] = '\0';
}

inline void blinkLed(const int times = 1, const int delayMs = 100)
{
    for (int i = 0; i < times; ++i)
    {
        digitalWrite(LED_BUILTIN, HIGH);
        delay(delayMs);
        digitalWrite(LED_BUILTIN, LOW);
        if (i != times - 1)
            delay(delayMs);
    }
}

inline void sendMsg(MSG &msg, const bool withCRC = true)
{
    if (withCRC)
    {
        const unsigned char crc8 = calcCRC8(reinterpret_cast<unsigned char *>(&msg), sizeof(MSG) - sizeof(msg.crc));
        msg.crc = crc8;
    }
    Serial.write(reinterpret_cast<char *>(&msg), sizeof(MSG));
    Serial.flush();
}

bool receiveMsg()
{
    if (Serial.available())
    {
        const size_t bytesRead = Serial.readBytes(reinterpret_cast<char *>(&mMsg), sizeof(MSG));
        if (bytesRead != sizeof(MSG))
        {
            createReplyMsg(mMsg, REPLY_INVALID_SIZE);
            sendMsg(mMsg, false);
            return false;
        }

        const unsigned char crc = calcCRC8(
            reinterpret_cast<unsigned char *>(&mMsg),
            sizeof(MSG) - sizeof(mMsg.crc)
        );
        if (crc != mMsg.crc)
        {
            createReplyMsg(mMsg, REPLY_BAD_CRC);
            sendMsg(mMsg, false);
            return false;
        }

        return true;
    }

    return false;
}

void sendWithRetry(MSG &msg)
{
    MSG copy;
    memcpy(&copy, &msg, sizeof(MSG));
    sendMsg(msg);

    // const unsigned long start = millis();
    unsigned int retry = 0;
    do
    {
        if (receiveMsg() && msg.type == MSG_REPLY)
        {
            const REPLY_DATA *pReply = reinterpret_cast<REPLY_DATA *>(msg.data);
            if (pReply->type == REPLY_OK)
                break;
            if (pReply->type == REPLY_BAD_CRC || pReply->type == REPLY_INVALID_SIZE)
                sendMsg(copy);
            retry++;
        }
        delay(RETRY_DELAY);
    // } while (millis() - start < RETRY_TIMEOUT || retry < MAX_RETRY);
    } while (retry < MAX_RETRY);
}

void setup()
{
    Serial.begin(9600);
    Serial.setTimeout(1500);

    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(RX_PWR_PIN, OUTPUT);
    pinMode(TX_PWR_PIN, OUTPUT);

    digitalWrite(RX_PWR_PIN, LOW);
    digitalWrite(TX_PWR_PIN, LOW);

    blinkLed(3);

    memset(&mMsg, 0, sizeof(MSG));
    mMsg.type = MSG_READY;
    sendMsg(mMsg);
}

inline void transmitRCData(const RC_DATA *data)
{
    mRCSwitch.setProtocol(data->proto);
    mRCSwitch.setRepeatTransmit(data->repeat);
    mRCSwitch.enableTransmit(TX_DATA_PIN);

    digitalWrite(LED_BUILTIN, HIGH);
    digitalWrite(TX_PWR_PIN, HIGH);
    delay(SWITCH_PWR_DELAY);
    mRCSwitch.send(data->code, data->length);
    digitalWrite(TX_PWR_PIN, LOW);
    digitalWrite(LED_BUILTIN, LOW);

    mRCSwitch.disableTransmit();
}

void listenRCMode()
{
    mRCSwitch.enableReceive(digitalPinToInterrupt(RX_DATA_PIN));
    mRCSwitch.setReceiveTolerance(60);

    digitalWrite(LED_BUILTIN, HIGH);
    digitalWrite(RX_PWR_PIN, HIGH);
    delay(SWITCH_PWR_DELAY);

    bool stop = false;
    RC_DATA data;

    do
    {
        if (mRCSwitch.available())
        {
            memset(&data, 0, sizeof(RC_DATA));
            data.code = mRCSwitch.getReceivedValue();
            data.length = mRCSwitch.getReceivedBitlength();
            mRCSwitch.getReceivedProtocolData(data.proto);
            createReplyMsg(mMsg, data);
            sendWithRetry(mMsg);
            mRCSwitch.resetAvailable();
        }

        if (receiveMsg() && mMsg.type == MSG_STOP)
            stop = true;
    } while (!stop);

    digitalWrite(RX_PWR_PIN, LOW);
    digitalWrite(LED_BUILTIN, LOW);

    mRCSwitch.disableReceive();
}

void loop()
{
    if (receiveMsg())
    {
        RC_DATA *pData;
        switch (mMsg.type)
        {
            case MSG_RX_REQUEST:
                createReplyMsg(mMsg, REPLY_OK);
                sendMsg(mMsg, false);
                listenRCMode();
                createReplyMsg(mMsg, REPLY_OK);
                sendMsg(mMsg, false);
                break;
            case MSG_TX_REQUEST:
                pData = reinterpret_cast<RC_DATA *>(mMsg.data);
                transmitRCData(pData);
                createReplyMsg(mMsg, REPLY_OK);
                sendMsg(mMsg, false);
                break;
            default:
                createReplyMsg(mMsg, REPLY_INVALID_MSG);
                sendMsg(mMsg, false);
                break;
        }
    }
}
