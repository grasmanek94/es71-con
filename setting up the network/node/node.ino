//source https://github.com/andrewrapp/xbee-arduino/blob/master/examples/ZdpScan/ZdpScan.ino

#include "XBee.h"
#include "Printers.h"
#include "zigbee.h"
#include "RunningMedian.h"

#ifndef lengthof
#define lengthof(x) (sizeof(x)/sizeof(*x))
#endif

struct node_info {
  XBeeAddress64 addr64;
  uint16_t addr16;
  uint8_t type: 2;
  uint8_t visited: 1;
  uint8_t rssi;
};

int nodes_found;
float startTime = 0;

node_info nodes[10];
XBeeWithCallbacks xbee;
RunningMedian samples = RunningMedian(10);

ZBExplicitTxRequest buildZdoRequest(XBeeAddress64 addr, uint16_t cluster_id, uint8_t *payload, size_t len) {
  ZBExplicitTxRequest tx(addr, payload, len);
  tx.setSrcEndpoint(WPAN_ENDPOINT_ZDO);
  tx.setDstEndpoint(WPAN_ENDPOINT_ZDO);
  tx.setClusterId(cluster_id);
  tx.setProfileId(WPAN_PROFILE_ZDO);
  tx.setFrameId(xbee.getNextFrameId());
  return tx;
}

uint8_t getNextTransactionId() {
  static uint8_t id = 0;
  return id++;
}

bool getAtValue(uint8_t cmd[2], uint8_t *buf, size_t len, uint16_t timeout = 150)
{
  AtCommandRequest req(cmd);
  req.setFrameId(xbee.getNextFrameId());
  uint8_t status = xbee.sendAndWait(req, timeout);
  if (status != 0) {
    Serial.print(F("Failed to read "));
    Serial.write(cmd, 2);
    Serial.print(F(" command. Status: 0x"));
    Serial.println(status, HEX);
    return false;
  }

  AtCommandResponse response;
  xbee.getResponse().getAtCommandResponse(response);
  if (response.getValueLength() != len) {
    Serial.print(F("Unexpected response length in "));
    Serial.write(cmd, 2);
    Serial.println(F(" response"));
    return false;
  }

  memcpy(buf, response.getValue(), len);
  return true;
}

// Invert the endianness of a given buffer
void invertEndian(uint8_t *buf, size_t len) {
  for (uint8_t i = 0, j = len - 1; i < len / 2; ++i, j--) {
    uint8_t tmp = buf[i];
    buf[i] = buf[j];
    buf[j] = tmp;
  }
}

void scan_network() {
  Serial.println();
  Serial.println("Discovering devices");
  // Fetch our operating PAN ID, to filter the LQI results
  uint8_t pan_id[8];
  getAtValue((uint8_t*)"OP", pan_id, sizeof(pan_id));
  // XBee sends in big-endian, but ZDO requests use little endian. For
  // easy comparsion, convert to little endian
  invertEndian(pan_id, sizeof(pan_id));

  // Fetch the addresses of the local node
  XBeeAddress64 local;
  uint8_t shbuf[4], slbuf[4], mybuf[2];
  if (!getAtValue((uint8_t*)"SH", shbuf, sizeof(shbuf)) ||
      !getAtValue((uint8_t*)"SL", slbuf, sizeof(slbuf)) ||
      !getAtValue((uint8_t*)"MY", mybuf, sizeof(mybuf)))
    return;

  nodes[0].addr64.setMsb((uint32_t)shbuf[0] << 24 | (uint32_t)shbuf[1] << 16 | (uint32_t)shbuf[2] << 8 | shbuf[3]);
  nodes[0].addr64.setLsb((uint32_t)slbuf[0] << 24 | (uint32_t)slbuf[1] << 16 | (uint32_t)slbuf[2] << 8 | slbuf[3]);
  nodes[0].addr16 = (uint16_t)mybuf[0] << 8 | mybuf[1];
  nodes[0].type = ZDO_MGMT_LQI_REQ_TYPE_UNKNOWN;
  nodes[0].visited = false;
  nodes_found = 1;

  Serial.print(F("0) 0x"));
  printHex(Serial, nodes[0].addr64);
  Serial.print(F(" (0x"));
  printHex(Serial, nodes[0].addr16);
  Serial.println(F(", Self)"));

  // nodes[0] now contains our own address, the rest is invalid. We
  // explore the network by asking for LQI info (neighbour table).
  // Initially, this pretends to send a packet to ourselves, which the
  // XBee firmware conveniently handles by pretending that a reply was
  // received (with one caveat: it seems the reply arrives _before_ the
  // TX status).
  uint8_t next = 0;
  do {
    // Query node i for its LQI table
    zdo_mgmt_lqi_req_t payload = {
      .transaction = getNextTransactionId(),
      .start_index = 0,
    };

    do {
      ZBExplicitRxResponse rx;
      if (!handleZdoRequest(F("requesting LQI/neighbour table"),
                            rx, nodes[next].addr64, ZDO_MGMT_LQI_REQ,
                            (uint8_t*)&payload, sizeof(payload)))
        break;
      //nodes[next].rssi = rx.getRssi();
      nodes[next].rssi = 0;
      zdo_mgmt_lqi_rsp_t *rsp = (zdo_mgmt_lqi_rsp_t*)(rx.getFrameData() + rx.getDataOffset());
      if (rsp->status != 0) {
        if (rsp->status != ZDO_STATUS_NOT_SUPPORTED) {
          Serial.print(F("LQI query rejected by 0x"));
          printHex(Serial, nodes[next].addr16);
          Serial.print(F(". Status: 0x"));
          printHex(Serial, rsp->status);
          Serial.println();
        }
        break;
      }

      if (rsp->start_index != payload.start_index) {
        Serial.println(F("Unexpected start_index, skipping this node"));
        break;
      }

      for (uint8_t i = 0; i < rsp->list_count; ++i) {
        zdo_mgmt_lqi_entry_t *e = &rsp->entries[i];
        node_info *n = &nodes[nodes_found];

        if (memcmp(&e->extended_pan_id_le, &pan_id, sizeof(pan_id)) != 0) {
          Serial.println(F("Ignoring node in other PAN"));
          continue;
        }

        // Skip if we know about this node already
        uint8_t dup;
        for (dup = 0; dup < nodes_found; ++dup) {
          if (nodes[dup].addr16 == e->nwk_addr_le)
            break;
        }
        if (dup != nodes_found)
          continue;

        n->addr64.setMsb(e->extended_addr_le >> 32);
        n->addr64.setLsb(e->extended_addr_le);
        n->addr16 = e->nwk_addr_le;
        n->type = e->flags0 & 0x3;

        Serial.print(nodes_found);
        Serial.print(F(") 0x"));
        printHex(Serial, n->addr64);
        Serial.print(F(" (0x"));
        printHex(Serial, n->addr16);
        switch (n->type) {
          case ZDO_MGMT_LQI_REQ_TYPE_COORDINATOR:
            Serial.println(F(", Coordinator)"));
            break;
          case ZDO_MGMT_LQI_REQ_TYPE_ROUTER:
            Serial.println(F(", Router)"));
            break;
          case ZDO_MGMT_LQI_REQ_TYPE_ENDDEVICE:
            Serial.println(F(", End device)"));
            break;
          case ZDO_MGMT_LQI_REQ_TYPE_UNKNOWN:
            Serial.println(F(", Unknown)"));
            break;
        }
        nodes_found++;

        if (nodes_found == lengthof(nodes)) {
          Serial.println(F("Device table full, terminating network scan"));
          return;
        }
      }

      // Got all neighbours available? Done.
      if (rsp->start_index + rsp->list_count >= rsp->table_entries)
        break;
      // More left? Loop and get more.
      payload.start_index += rsp->list_count;
      payload.transaction = getNextTransactionId();
    } while (true);

    // Done with this node, on to the next
    nodes[next].visited = true;
    ++next;
  } 
  while (next < nodes_found);
  Serial.println(F("Finished scanning"));
  Serial.println(F("Press a number to scan that node, or press r to rescan the network"));
}
bool matchZdoReply(ZBExplicitRxResponse& rx, uintptr_t data) {
  uint8_t *payload = rx.getFrameData() + rx.getDataOffset();
  uint8_t transactionId = (uint8_t)data;

  return rx.getSrcEndpoint() == 0 &&
         rx.getDstEndpoint() == 0 &&
         rx.getProfileId() == WPAN_PROFILE_ZDO &&
         payload[0] == transactionId;
}
bool handleZdoRequest(const __FlashStringHelper *msg, ZBExplicitRxResponse& rx, XBeeAddress64 addr, uint16_t cluster_id, uint8_t *payload, size_t len) {
  ZBExplicitTxRequest tx = buildZdoRequest(addr, cluster_id, (uint8_t*)payload, len);
  xbee.send(tx);

  uint8_t transaction_id = payload[0];
  // This waits up to 5000 seconds, since the default TX timeout (NH
  // value of 1.6s, times three retries) is 4.8s.
  uint8_t status = xbee.waitFor(rx, 5000, matchZdoReply, transaction_id, tx.getFrameId());
  switch(status) {
    case 0: // Success
      return true;
    case XBEE_WAIT_TIMEOUT:
      Serial.print(F("No reply received from 0x"));
      printHex(Serial, addr.getMsb());
      printHex(Serial, addr.getLsb());
      Serial.print(F(" while "));
      Serial.print(msg);
      Serial.println(F("."));
      return false;
    default:
      Serial.print(F("Failed to send to 0x"));
      printHex(Serial, addr.getMsb());
      printHex(Serial, addr.getLsb());
      Serial.print(F(" while "));
      Serial.print(msg);
      Serial.print(F(". Status: 0x"));
      printHex(Serial, status);
      Serial.println();
      return false;
  }
}

void blinkLed()
{
  digitalWrite(LED_BUILTIN, LOW);
  if(nodes_found>0)
  {
    double delayTime = 10000.0/((double)nodes_found*2.0);
    startTime = millis();
    
    for(int i = 0; i < nodes_found; ++i)
    {
        while(millis()<(startTime+delayTime))
        {
            delay(1);
        }
        startTime = millis();
        digitalWrite(LED_BUILTIN,0);
        while(millis()<(startTime+delayTime))
        {
            delay(1);
        }
        startTime = millis();
        digitalWrite(LED_BUILTIN,1);
    }
    
    /*samples.clear();
    for(int i = 0; i < nodes_found; ++i)
    {
        samples.add(nodes[i].rssi);
    }
    float mrssi = samples.getMedian();
    // set red if bad mean rssi
    digitalWrite(5,mrssi < 100);*/
  }
}

void setup()
{
  pinMode(LED_BUILTIN,OUTPUT);
}

void loop()
{
  nodes_found = 0;
  scan_network();
  blinkLed();

}
