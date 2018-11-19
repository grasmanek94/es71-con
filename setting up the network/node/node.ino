//source https://github.com/andrewrapp/xbee-arduino/blob/master/examples/ZdpScan/ZdpScan.ino

#include "XBee.h"
#include "Printers.h"

int nodes_found;

node_info nodes[10];
XBeeWithCallbacks xbee;

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

void blink()
{

}

void setup()
{
}

void loop()
{
  nodes_found = 0;
  scan_network();

}
