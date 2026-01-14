#include <Arduino.h>
#include "UserPins.h"  // User's Pins Mapping

#if __has_include("STM32FreeRTOS.h")
#  include "hal_rtos.h"
#endif

/* ================= CONFIG ================= */

#define TEAM_ID "1043"

/* ================= UART ================= */

HardwareSerial Xbee(USER_GPIO_XBEE_RX, USER_GPIO_XBEE_TX);

/* ================= SHARED DATA ================= */

String   rx_message;
String   tx_buf;
uint32_t packet_count = 0;

bool     simEnabled        = false;
bool     simActivated      = false;
uint32_t simPressure       = 101325;
bool     telemetry_enabled = true;

hal::rtos::mutex_t mtx_uart;

/* ================= USER DATA ================= */

struct {
  String   cmd_echo;
  float    altitude;
  uint32_t last_ack;
  uint32_t last_nack;
} data;

/* ================= INIT ================= */

void UserSetupUSART() {
  Xbee.begin(115200);
}

/* ================= COMMAND HANDLER ================= */

void HandleCommand(const String &rx) {
  // CMD header check
  if (rx.substring(0, 9) != "CMD,1043,") {
    Serial.println("NACK");
    return;
  }

  String cmd = rx.substring(9);
  cmd.trim();

  data.cmd_echo = cmd;

  /* ===== COMMANDS ===== */

  /* ========== CX ========== */
  if (cmd == "CX,ON") {
    telemetry_enabled = true;
  } else if (cmd == "CX,OFF") {
    telemetry_enabled = false;
  }

  /* ========== ST ========== */
  // else if (cmd.substring(0, 3) == "ST,") {
  //   String arg = cmd.substring(3);
  //   arg.trim();

  //   if (arg == "GPS")
  //     missionTime = "GPS_TIME";  // hook GPS later
  //   else
  //     missionTime = arg;
  // }

  /* ========== SIM ========== */
  else if (cmd == "SIM,ENABLE") {
    simEnabled = true;
  } else if (cmd == "SIM,ACTIVATE") {
    if (simEnabled)
      simActivated = true;
  } else if (cmd == "SIM,DISABLE") {
    simEnabled   = false;
    simActivated = false;
  }

  /* ========== SIMP ========== */
  else if (cmd.substring(0, 5) == "SIMP,") {
    if (simEnabled && simActivated) {
      simPressure = cmd.substring(5).toInt();
    }
  }

  /* ========== CAL ========== */
  else if (cmd == "CAL") {
    Serial.println("Altitude calibrated to 0 m");
  }

  /* ========== MEC ========== */
  else if (cmd == "MEC,PL,ON") {
    Serial.println("Payload Release ON");
  } else if (cmd == "MEC,PL,OFF") {
    Serial.println("Payload Release OFF");
  } else if (cmd == "MEC,INS,ON") {
    Serial.println("Instrument Deploy ON");
  } else if (cmd == "MEC,INS,OFF") {
    Serial.println("Instrument Deploy OFF");
  } else if (cmd == "MEC,PAR,ON") {
    Serial.println("Paraglider Rotation ON");
  } else if (cmd == "MEC,PAR,OFF") {
    Serial.println("Paraglider Rotation OFF");
  }

  /* ========== RESET ========== */
  else if (cmd == "RESET") {
    __NVIC_SystemReset();
  }

  /* ========== UNKNOWN ========== */
  else {
    Serial.print("Unknown CMD: ");
    Serial.println(cmd);
    ++data.last_nack;
    --data.last_ack;
  }
}

/* ================= RX THREAD ================= */

void CB_RxCommand(void *) {
  hal::rtos::interval_loop(5ul, [&]() -> void {
    mtx_uart.exec([&]() -> void {
      while (Xbee.available()) {
        char c = Xbee.read();

        if (c == '\n') {
          rx_message.trim();
          HandleCommand(rx_message);
          rx_message = "";
        } else {
          rx_message += c;
        }
      }
    });
  });
}

/* ================= TX THREAD ================= */

void CB_TxTelemetry(void *) {
  hal::rtos::interval_loop(1000ul, [&]() -> void {
    mtx_uart.exec([&]() -> void {
      tx_buf = "";
      tx_buf += TEAM_ID;
      tx_buf += ",";
      tx_buf += packet_count++;
      tx_buf += ",";
      tx_buf += String(data.altitude, 1);
      tx_buf += ",";
      tx_buf += data.cmd_echo;
      
      if (telemetry_enabled) {
        Xbee.println(tx_buf);
      }
    });
  });
}

/* ================= THREAD REG ================= */

void UserThreads() {
  hal::rtos::scheduler.create(
    CB_RxCommand,
    {.name = "RX_CMD", .stack_size = 8192, .priority = osPriorityNormal});

  hal::rtos::scheduler.create(
    CB_TxTelemetry,
    {.name = "TX_TLM", .stack_size = 8192, .priority = osPriorityHigh});
}

/* ================= SETUP ================= */

void setup() {
  Serial.begin(115200);
  UserSetupUSART();

  hal::rtos::scheduler.initialize();
  UserThreads();
  hal::rtos::scheduler.start();
}

void loop() {}
