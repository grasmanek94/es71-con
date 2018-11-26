#include "XBee.h"
#include <SoftwareSerial.h>
#include "Printers.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

// Notes
// for neighbour discovery some inspiration taken from https://github.com/msepcot/arduino_samples/blob/master/XBeeNodeDiscovery/XBeeNodeDiscovery.pde

// Define SoftSerial TX/RX pins
// Connect Arduino pin 8 to TX of usb-serial device
uint8_t ssRX = 8;
// Connect Arduino pin 9 to RX of usb-serial device
uint8_t ssTX = 9;
// Remember to connect all devices to a common Ground: XBee, Arduino and USB-Serial device
SoftwareSerial nss(ssRX, ssTX);

XBee xbee = XBee();
AtCommandRequest request = AtCommandRequest();
AtCommandResponse response = AtCommandResponse();

uint8_t ND[] = {'N', 'D'}; // Node Discover
// uint8_t NT[] = {'N', 'T'}; // Node Discover Timeout
uint8_t icu_payload[] = { 'I', 'C', 'U' }; // "I Choose U" - listener needs to check for this and stay awake

int timeout = 3000;

long randomNumber = 0L;

#define DEFAULT_FRAME_ID 0 // disable explicit RX packets

// we need to store the ids of the neighbours in something
// the problem is I'm not sure how long to make this as we
// have to declare a length up front because it's C
String neighbour_ids = String(160); // FIXME: only enough for 20 neighbour addresses
int neighbour_counter = 0;

// initialize sum and weight for gossip
float gossip_sum = 25.0; // TODO: read this from sensors
float gossip_weight = 1.0;

// Function taken from http://preshing.com/20111007/how-to-generate-random-timings-for-a-poisson-process/
// Adapted for Arduino at http://forum.arduino.cc/index.php?topic=292956.0
float nextTime(float rateParameter)
{
  // usage nextTime(1/10.0) where 10 is the desired number of miliseconds
  return -log(1.0f - random(RAND_MAX) / ((float)RAND_MAX + 1)) / rateParameter;
}

void setup() { 
  // Start the serial port
  Serial.begin(9600);  

  Serial.print("Setup...");
  
  // put your setup code here, to run once:
  pinMode (3,OUTPUT);    // put XBee to sleep0
  digitalWrite(3,HIGH);
  delay(5000);
  digitalWrite(3,LOW); // wake up any attached xbees

  // start soft serial
  nss.begin(9600);
  // Tell XBee to use Hardware Serial. It's also possible to use SoftwareSerial
  xbee.setSerial(nss);

  // taken from https://arduino-info.wikispaces.com/Listing_4_4_HeadsTailsExample
  randomSeed(analogRead(A0)); // This seeds the random number generator

  delay(1000);

  Serial.println(" done.");
}

void loop() {
  // ## idea for simulation - reset every minute
  // choose random temperature between 10 and 30 degrees
  // threshold of 20
  // if high/low, set own LEDs
  // ## end simulation idea

  // Preliminary steps
  // setup Xbee coordinator (probably using XCTU)
  // setup Xbee end device (probably using XCTU)

  // steps
  // sleep for poisson timing of around 40ms
  delay((long)nextTime(1 / 100.0));

  //   flip a coin - either listen or broadcast
  int mode = (rand() % 2);

  if (mode == 0) { //then broadcast
    Serial.println("BROADCASTER");
    // check for neighbours
    request.setCommand(ND);
    xbee.send(request);

    // Try just writing the ND packet directly
    // byte message[] = {0x7E, 0x00, 0x04, 0x08, 0x01, 0x4E, 0x44, 0x64 };
    // nss.write(message, sizeof(message));

    // Serial.print(7E 00 04 08 01 4E 44 64)
    //delay(60); // give neighbours time to respond

    // clear out list of neighbour ids
    // memset(neighbour_ids, 0, sizeof neighbour_ids); // tried this but doesn't seem to work
    // FIXME: only enough for 20 neighbour addresses
    neighbour_ids = "";
    neighbour_counter = 0;

    // TODO: send chosen partner message here
    // adapted from https://github.com/andrewrapp/xbee-arduino/blob/3e9eb4303d20c0f590426db83d77d375c914651d/examples/Series2_Tx/Series2_Tx.pde

    while (xbee.readPacket(timeout)) {
      // should be receiving AT command responses
      //      Serial.print("\r\nIsAvailabe\r\n");
      //      Serial.print(xbee.getResponse().isAvailable(), DEC);
      //      Serial.print("\r\nIsError\r\n");
      //      Serial.print(xbee.getResponse().isError(), DEC);
      //      Serial.print("\r\ngetApiId\r\n");
      //      Serial.print(xbee.getResponse().getApiId(), HEX);
      //      Serial.print("\r\n");

      if (xbee.getResponse().getApiId() == AT_RESPONSE) {
        xbee.getResponse().getAtCommandResponse(response);
        if (response.isOk()) {
          // parse response - get neighbour's high and low ids
          // explanation of AT_RESPONSE frame here https://stackoverflow.com/questions/26677590/xbee-node-discovery-response
          // sample response 00 00 00 13 A2 00 40 A4 75 28 20 00 FF FE 00 00 C1 05 10 1E
          // 2 bytes for network address 00 00 (means it was the coordinator responding I think)
          // four bytes for serial number high
          // four bytes for serial number low
          for (int i = 0; i < 8; i++) {
            if (response.getValue()[i + 2] < 10) {
              neighbour_ids += ("0" + String(response.getValue()[i + 2], HEX)); // two here is the offset from the start of the received message
            } else {
              neighbour_ids += String(response.getValue()[i + 2], HEX);
            }
          }
          
          neighbour_ids += "\n";
          
          Serial.print("Neighbour list: \n");
          Serial.print(neighbour_ids);
          Serial.print("\n");

          neighbour_counter++;

        } else {
          Serial.print("No neighbour responses received\n");
        }
      } else {
        Serial.print("No AT command response received\n");
      }

      delay(timeout); // don't look for neighbours forever
    }

    // choose partner node at random - send them a "chosen partner" message, wait for ack
    Serial.print("\nReceived ID count: ");
    Serial.print(neighbour_counter);
    Serial.print("\n");

    if (neighbour_counter > 0) {
      int random_id_offset = (rand() % neighbour_counter) * 16; // TODO: apparently not true random https://stackoverflow.com/a/1203783/2618015

      //char chosen_address[16];
      //neighbour_ids.substring(random_id_offset, random_id_offset + 16).toCharArray(chosen_address, 17);
      //long unsigned chosen_address_64_ul = getUInt64fromHex(chosen_address);
      //print64(getUInt64fromHex(chosen_address)); // yiedls 5526146525721898 for 0013A20040A4752A
      //Serial.println(chosen_address);

      uint32_t chosen_address_upper;
      char chosen_address_upper_chars[8];
      neighbour_ids.substring(random_id_offset, random_id_offset + 8).toCharArray(chosen_address_upper_chars, 9);
      sscanf(chosen_address_upper_chars, "%lx", &chosen_address_upper);
      //
      // the leading zeros in 0013A200 aren't handled well by sscanf and the
      // result is translated to FE13A200 which appears to be reading from another memory location
      // In practice our upper addresses are all from the same manufacturer (xbee)
      // so I've had to hard code this for now
      // chosen_address_upper = 1286656; // 0013A200 as int

      //Serial.print("Upper: " + String(chosen_address_upper_chars) + "\n");
      //Serial.print((unsigned int)chosen_address_upper, DEC);

      // this would also fail if the lower address began with leading zeros
      long chosen_address_lower;
      char chosen_address_lower_chars[8];
      neighbour_ids.substring(random_id_offset + 8, random_id_offset + 16).toCharArray(chosen_address_lower_chars, 9);
      sscanf(chosen_address_lower_chars, "%lx", &chosen_address_lower);

      Serial.print("Lower: " + String(chosen_address_lower_chars) + "\n");
      Serial.println(chosen_address_lower);

      // chosen_address_upper = 1286656;
      // chosen_address_lower = 1097006828; // router
      // chosen_address_lower = 1084519722; // end device
      // chosen_address_lower = 1084519720; // coordinator

      // TODO: send chosen partner message here
      // adapted from https://github.com/andrewrapp/xbee-arduino/blob/3e9eb4303d20c0f590426db83d77d375c914651d/examples/Series2_Tx/Series2_Tx.pde
      XBeeAddress64 addr64 = XBeeAddress64(chosen_address_upper, chosen_address_lower);

      Serial.println("64 bit address as int");
      print64(addr64.get());

      ZBTxRequest zbTx = ZBTxRequest(addr64, icu_payload, sizeof(icu_payload));
      ZBTxStatusResponse txStatus = ZBTxStatusResponse();

      xbee.send(zbTx);

      // after sending a tx request, we expect a status response
      // wait up to half second for the status response
      if (xbee.readPacket(timeout)) {
        // got a response!
        Serial.println("Received response from neighbour\n");

        // should be a znet tx status
        if (xbee.getResponse().getApiId() == ZB_TX_STATUS_RESPONSE) {
          xbee.getResponse().getZBTxStatusResponse(txStatus);

          if (txStatus.getDeliveryStatus() == SUCCESS) {
            // success
            // send (1/2 weight, 1/2 sum) to chosen neighbour
            // we are going to send two floats of 4 bytes each in length
            uint8_t gossip_payload[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };

            // technique to send floats in payload adapted from http://streylab.com/blog/2012/10/14/sending-humidity-and-temperature-data-with-zigbee.html
            // union to convery float to byte string
            union u_tag {
              uint8_t b[4];
              float fval;
            } u;

            // convert sum into a byte array and copy it into the payload array
            u.fval = gossip_sum / 2;
            for (int i = 0; i < 4; i++) {
              gossip_payload[i] = u.b[i];
            }

            // same for the weight
            u.fval = gossip_weight / 2;
            for (int i = 0; i < 4; i++) {
              gossip_payload[i + 4] = u.b[i];
            }

            // TODO - make this more robust by checking that the message was received
            ZBTxRequest gossip_tx = ZBTxRequest(addr64, gossip_payload, sizeof(gossip_payload));
            // ZBTxStatusResponse gossip_tx_status = ZBTxStatusResponse();
            xbee.send(gossip_tx);

            Serial.println("Broadcaster:");
            Serial.print("Sum: ");
            Serial.println(gossip_sum, 2);
            Serial.print("Weight: ");
            Serial.println(gossip_weight, 2);
            Serial.print("Temperature: ");
            Serial.println((gossip_sum / gossip_weight), 2);

            // send it to myself i.e. set weight = (self.weight + weight/2), sum = (self.sum + sum/2)
            gossip_sum = (gossip_sum + gossip_sum / 2);
            gossip_weight = (gossip_weight + gossip_weight / 2);

            // sleep for timeout to ensure that no more packets are received
            delay(timeout);
          } else {
            // the remote XBee did not receive our packet. is it powered on?
            Serial.println("Message not delivered successfully");
          }
        } else if (xbee.getResponse().isError()) {
          Serial.println("Error reading packet.  Error code: ");
          Serial.println(xbee.getResponse().getErrorCode());
        }
      }
    } // end if(neighbour_counter > 0)
  } else { // listen
    Serial.print("LISTENER\n");
    Serial.println("");
    // TODO
    // respond to neighbour messages requests - already handled by Xbee
    Rx64Response chosen_partner_rx64 = Rx64Response();
    Rx64Response sum_weight_rx64 = Rx64Response();

    // wait for a "chosen partner" message, send ack (ack already handled by Xbee)
    while (xbee.readPacket(timeout)) {

      if (xbee.getResponse().isAvailable()) {
        Serial.print("\r\nIsAvailabe\r\n");
        Serial.print(xbee.getResponse().isAvailable(), DEC);
        Serial.print("\r\nIsError\r\n");
        Serial.print(xbee.getResponse().isError(), DEC);
        Serial.print("\r\ngetApiId\r\n");
        Serial.print(xbee.getResponse().getApiId(), HEX);
        Serial.print("\r\n");

        if (xbee.getResponse().getApiId() == ZB_RX_RESPONSE) { // is it a request or response? maybe RX_64_RESPONSE?
          xbee.getResponse().getRx64Response(chosen_partner_rx64);

          uint8_t payload[3];
          for (int i = 0; i < 3; i++) {
            payload[i] = chosen_partner_rx64.getData(i + 7); // offset into the packet
          }

          // would rather compare received payload directly but this
          // memcmp(payload, icu_payload, 3)
          // didn't seem to work

          // Serial.println(chosen_partner_rx64.getDataLength());

          if (chosen_partner_rx64.getDataLength() == (1 + sizeof(icu_payload))) {
            Serial.println("Received chosen neighbour packet");

            while (xbee.readPacket(timeout)) {
              Serial.print("\r\nGossip IsAvailabe\r\n");
              Serial.print(xbee.getResponse().isAvailable(), DEC);
              Serial.print("\r\nGossip IsError\r\n");
              Serial.print(xbee.getResponse().isError(), DEC);
              Serial.print("\r\nGossip getApiId\r\n");
              Serial.print(xbee.getResponse().getApiId(), HEX);
              Serial.print("\r\n");

              if (xbee.getResponse().isAvailable()) {
                if (xbee.getResponse().getApiId() == ZB_RX_RESPONSE) {
                  //   hold the door open (don't hang up or go to sleep)
                  //   wait for sum/weight message and set sum = (self.sum + msg.sum), weight = (self.weight + msg.weight)

                  xbee.getResponse().getRx64Response(sum_weight_rx64);
                  Serial.println("Gossip packet size:");
                  Serial.println(sizeof(sum_weight_rx64.getDataOffset()), DEC);
                  Serial.println("Gossip data size:");
                  Serial.println(sum_weight_rx64.getDataLength(), DEC);

                  // check length of payload
                  if (sum_weight_rx64.getDataLength() == (8 + 1)) { // 1 is checksum presumably

                    // we are going to receive two floats of 4 bytes each in length

                    union sum_type {
                      uint8_t b[4];
                      float fval;
                    } sum;

                    union weight_type {
                      uint8_t b[4];
                      float fval;
                    } weight;

                    for (int i = 0; i < 4; i++) {
                      sum.b[i] = sum_weight_rx64.getData(i + 1);
                    }

                    for (int i = 0; i < 4; i++) {
                      weight.b[i] = sum_weight_rx64.getData(i + 4 + 1);
                    }

                    gossip_sum = (gossip_sum + sum.fval); // check algorithm if this is correct
                    gossip_weight = (gossip_weight + weight.fval); // check algorithm if this is correct

                    Serial.println("Listener:");
                    Serial.print("Sum: ");
                    Serial.println(gossip_sum, 2);
                    Serial.print("Weight: ");
                    Serial.println(gossip_weight, 2);
                    Serial.print("Temperature: ");
                    Serial.println((gossip_sum / gossip_weight), 2);
                  }
                }
              }
              // make sure no further packets are processed
              delay(timeout);
            }
          }
        }
      }
    }
    // go back to sleep if timeout is hit
  }

  // check against threshold and set LED

} // end loop

uint64_t getUInt64fromHex(char const *str)
{
  uint64_t accumulator = 0;
  for (size_t i = 0 ; isxdigit((unsigned char)str[i]) ; ++i)
  {
    char c = str[i];
    accumulator *= 16;
    if (isdigit(c)) /* '0' .. '9'*/
      accumulator += c - '0';
    else if (isupper(c)) /* 'A' .. 'F'*/
      accumulator += c - 'A' + 10;
    else /* 'a' .. 'f'*/
      accumulator += c - 'a' + 10;

  }

  return accumulator;
}

void print64(uint64_t value)
{
  const int NUM_DIGITS    = log10(value) + 1;

  char sz[NUM_DIGITS + 1];

  sz[NUM_DIGITS] =  0;
  for ( size_t i = NUM_DIGITS; i--; value /= 10)
  {
    sz[i] = '0' + (value % 10);
  }

  Serial.println(sz);
}
