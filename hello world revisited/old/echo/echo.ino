#include "XBee.h"
#include "Printers.h"

XBeeWithCallbacks xbee;

void zbReceive(ZBRxResponse& rx, uintptr_t)
{
    ZBTxRequest tx;
    tx.setAddress64(rx.getRemoteAddress64());
    tx.setAddress16(rx.getRemoteAddress16());
    tx.setPayload(rx.getFrameData() + rx.getDataOffset(), rx.getDataLength());
    
    xbee.send(tx);
    Serial.println(F("Sending ZBTxRequest"));
}

void receive16(Rx16Response& rx, uintptr_t)
{
    Tx16Request tx;
    tx.setAddress16(rx.getRemoteAddress16());
    tx.setPayload(rx.getFrameData() + rx.getDataOffset(), rx.getDataLength());
    
    xbee.send(tx);
    Serial.println(F("Sending Tx16Request"));
}

void receive64(Rx64Response& rx, uintptr_t) 
{
    Tx64Request tx;
    tx.setAddress64(rx.getRemoteAddress64());
    tx.setPayload(rx.getFrameData() + rx.getDataOffset(), rx.getDataLength());
    
    xbee.send(tx);
    Serial.println(F("Sending Tx64Request"));
}

void setup()
{
    Serial.begin(9600);
    xbee.setSerial(Serial);
    
    xbee.onZBRxResponse(zbReceive);
    xbee.onRx16Response(receive16);
    xbee.onRx64Response(receive64);
}

void loop() 
{
    xbee.loop();
}
