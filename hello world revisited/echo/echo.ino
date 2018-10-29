#include "XBee.h" 

XBee xbee = XBee();
XBeeResponse response = XBeeResponse();
ZBRxResponse rx = ZBRxResponse();

void setup()
{
	Serial.begin(9600);
	xbee.begin(Serial);
}

void loop()
{
	String sample;
	xbee.readPacket();
	if (xbee.getResponse().isAvailable())
	{
		Serial.println(xbee.getResponse().getApiId());
		if (xbee.getResponse().getApiId() == ZB_RX_RESPONSE)
		{
			xbee.getResponse().getZBRxResponse(rx);
			for (int i = 0; i < rx.getDataLength(); i++) 
			{
				sample += (char)rx.getData(i);
			}
			Serial.println(sample);
		}
	}
	else if (xbee.getResponse().isError())
	{
		Serial.println("Error reading packet. Error code: ");
		Serial.println(xbee.getResponse().getErrorCode());
	}
	delay(100);
}
