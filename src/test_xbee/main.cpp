#include <Arduino.h>
#include "UserPins.h"

/* ── XBee serial ── */
HardwareSerial Xbee(USER_GPIO_XBEE_RX, USER_GPIO_XBEE_TX);

/* ── Command state (mirrors main) ── */
bool     telemetry_enabled = false;
bool     simEnabled        = false;
bool     simActivated      = false;
uint32_t simPressure       = 0;
uint32_t last_ack          = 0;
uint32_t last_nack         = 0;
String   rx_message        = "";

/* ── Command handler (mirrors HandleCommand in main) ── */
void HandleCommand(const String &rx) {
  if (rx.substring(0, 9) != "CMD,1043,") {
    Serial.println("NACK");
    ++last_nack;
    return;
  }

  String cmd = rx.substring(9);
  cmd.trim();

  Serial.print("CMD: ");
  Serial.println(cmd);

  /* ========== CX ========== */
  if (cmd == "CX,ON") {
    telemetry_enabled = true;
    Serial.println("Telemetry ON");

  } else if (cmd == "CX,OFF") {
    telemetry_enabled = false;
    Serial.println("Telemetry OFF");

    /* ========== SIM ========== */
  } else if (cmd == "SIM,ENABLE") {
    simEnabled = true;
    Serial.println("SIM Enabled");

  } else if (cmd == "SIM,ACTIVATE") {
    if (simEnabled) {
      simActivated = true;
      Serial.println("SIM Activated");
    } else {
      Serial.println("SIM,ACTIVATE ignored: SIM not enabled");
    }

  } else if (cmd == "SIM,DISABLE") {
    simEnabled   = false;
    simActivated = false;
    Serial.println("SIM Disabled");

    /* ========== SIMP ========== */
  } else if (cmd.substring(0, 5) == "SIMP,") {
    if (simEnabled && simActivated) {
      simPressure = cmd.substring(5).toInt();
      Serial.print("SIMP: ");
      Serial.println(simPressure);
    } else {
      Serial.println("SIMP ignored: SIM not active");
    }

    /* ========== CAL ========== */
  } else if (cmd == "CAL") {
    Serial.println("Altitude calibrated to 0 m");

    /* ========== MEC ========== */
  } else if (cmd == "MEC,PL,ON") {
    Serial.println("MEC PL ON");
  } else if (cmd == "MEC,PL,OFF") {
    Serial.println("MEC PL OFF");
  } else if (cmd == "MEC,INS,ON") {
    Serial.println("MEC INS ON");
  } else if (cmd == "MEC,INS,OFF") {
    Serial.println("MEC INS OFF");
  } else if (cmd == "MEC,PAR,ON") {
    Serial.println("Paraglider Rotation ON");
  } else if (cmd == "MEC,PAR,OFF") {
    Serial.println("Paraglider Rotation OFF");

    /* ========== RESET ========== */
  } else if (cmd == "RESET") {
    Serial.println("Resetting...");
    delay(100);
    __NVIC_SystemReset();

    /* ========== UNKNOWN ========== */
  } else {
    Serial.print("Unknown CMD: ");
    Serial.println(cmd);
    ++last_nack;
    --last_ack;
    return;
  }

  ++last_ack;
  Serial.print("ACK=");
  Serial.print(last_ack);
  Serial.print("  NACK=");
  Serial.println(last_nack);
}

void setup() {
  Serial.begin(460800);
  delay(2000);

  Xbee.begin(115200);

  Serial.println("=== test_xbee: XBee receive test ===");
  Serial.print("XBee RX: PB5  TX: PB6  Baud: 9600");
  Serial.println();
  Serial.println("Waiting for commands...");
  Serial.println("Format: CMD,1043,<command>");
}

void loop() {
  // while (Xbee.available()) {
  //   char c = Xbee.read();

  //   if (c == '\n') {
  //     rx_message.trim();
  //     if (rx_message.length() > 0) {
  //       Serial.print("RX raw: ");
  //       Serial.println(rx_message);
  //       HandleCommand(rx_message);
  //     }
  //     rx_message = "";
  //   } else {
  //     rx_message += c;
  //   }
  // }
  Xbee.println("HEllo");
}
