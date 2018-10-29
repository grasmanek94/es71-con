#include "XBee.h"
#include "Printers.h"

XBeeWithCallbacks xbee;

XBeeAddress64 arduino[2] = {
    XBeeAddress64(0x0013A200, 0x40937BA8),
    XBeeAddress64(0x0013A200, 0x409382C6),
};

void SendString(XBeeAddress64 addr64, String data)
{
    xbee.send(ZBTxRequest(addr64, data.c_str(), data.length()));
}

void zbReceive(ZBRxResponse& rx, uintptr_t)
{
    String data;
    char* ptr = rx.getFrameData() + rx.getDataOffset();
    do
    {
        data += *ptr;
    }
    while(ptr++ != rx.getFrameData() + rx.getDataOffset() + rx.getDataLength())

    Serial.print("Received data from ");
    Serial.print(rx.getRemoteAddress64(), HEX);
    Serial.print(rx.getRemoteAddress16(), HEX);
    Serial.print(": ");
    Serial.println(data);
}

void receive16(Rx16Response& rx, uintptr_t)
{
    String data;
    char* ptr = rx.getFrameData() + rx.getDataOffset();
    do
    {
        data += *ptr;
    }
    while(ptr++ != rx.getFrameData() + rx.getDataOffset() + rx.getDataLength())

    Serial.print("Received data from ");
    Serial.print(rx.getRemoteAddress16(), HEX);
    Serial.print(": ");
    Serial.println(data);
}

void receive64(Rx64Response& rx, uintptr_t) 
{
    String data;
    char* ptr = rx.getFrameData() + rx.getDataOffset();
    do
    {
        data += *ptr;
    }
    while(ptr++ != rx.getFrameData() + rx.getDataOffset() + rx.getDataLength())

    Serial.print("Received data from ");
    Serial.print(rx.getRemoteAddress64(), HEX);
    Serial.print(": ");
    Serial.println(data);
}

void setup() 
{
    Serial.begin(9600);
    xbee.setSerial(Serial);
    
    xbee.onZBRxResponse(zbReceive);
    xbee.onRx16Response(receive16);
    xbee.onRx64Response(receive64);

    SendString(arduino[1], "Hello from arduino 1 to arduino 2!");
}

void loop()
{
    xbee.loop();
}
