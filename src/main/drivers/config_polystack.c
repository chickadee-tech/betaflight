
#include <stdbool.h>
#include <stdint.h>

#include <pb.h>
#include <pb_decode.h>

#include "platform.h"

// Begin headers to access masterConfig.
#include "common/axis.h"
#include "common/maths.h"

#include "sensors/sensors.h" // before sensor.h
#include "drivers/sensor.h" // before accgyro!
#include "drivers/accgyro.h"

#include "drivers/gpio.h" // before timer!
#include "drivers/timer.h" // before pwm_rx!
#include "drivers/pwm_rx.h"
#include "drivers/serial.h"

#include "sensors/acceleration.h"
#include "flight/failsafe.h"
#include "flight/pid.h" // before imu.!
#include "flight/imu.h"
#include "flight/mixer.h"
#include "flight/navigation.h"

#include "io/escservo.h"
#include "io/gimbal.h"
#include "io/gps.h"
#include "io/rc_controls.h"
#include "io/serial.h"

#include "sensors/barometer.h"
#include "sensors/battery.h"
#include "sensors/boardalignment.h"
#include "sensors/gyro.h"

#include "telemetry/telemetry.h"

#include "config/runtime_config.h"
#include "config/config.h"
#include "config/config_profile.h"
// End headers to access masterConfig.
#include "config/config_master.h"

#include "bus_i2c.h"
#include "config_polystack.h"
#include "config_polystack.pb.h"
#include "gpio.h"
#include "light_led.h"

#include "blackbox/blackbox_io.h"

static serialPortIdentifier_e serialPortMapping[8] = POLYSTACK_SERIAL_PORT_ORDER;

bool callback(pb_istream_t *stream, uint8_t *buf, size_t count)
{
  I2CMemoryStreamState* stream_state = (I2CMemoryStreamState*) stream->state;

  bool read_status = i2cReadMemory(POLYSTACK_I2C_INSTANCE, stream_state->device_address, stream_state->next_memory_address, count, buf);
  stream_state->next_memory_address += count;
  return read_status;
}

pb_istream_t i2cstream = {&callback, 0, SIZE_MAX};

bool polystackRead(uint8_t index, PolystackMod* mod_info) {
  // Read eeproms upwards.
  I2CMemoryStreamState stream_state = {(BASE_MEMORY_ADDRESS >> 1) + index, 0};
  pb_istream_t i2cstream = {&callback, &stream_state, 64 * 1024};
  // now read the proto
  bool read_status = pb_decode_delimited(&i2cstream, PolystackMod_fields, mod_info);
    // for (int i = 0; i < 10; i++) {
    //   LED1_TOGGLE;
    //   delay(200);
    // }
  return read_status;
}

bool polystackReadSerial(uint8_t index, uint8_t serial_number[16]) {
  return i2cReadMemory(POLYSTACK_I2C_INSTANCE, (BASE_SERIAL_DEVICE_ADDRESS >> 1) + index, SERIAL_WORD_ADDRESS, 16, serial_number);
}

void polystackAutoConfigure(void) {
  PolystackMod mod = PolystackMod_init_default;
  // for (int i = 0; i < 10; i++) {
  //   LED2_TOGGLE;
  //   delay(50);
  // }
  uint8_t serial_index = 0;
  for (int i = 1; i < 8; i++) {
    if (!polystackRead(i, &mod)) {
      break;
    }
    // TODO(tannewt): When we need to adapt functionality based on module id
    // tweak the PolystackMod data structure instead of the Cleanflight config
    // directly. That will ensure the datastructure is adequate for subsequent
    // mods.

    // Configure serial ports based on the protos stored in mod memory.
    for (int i = 0; i < mod.serial_config_count; ++i) {
      serialPortConfig_t* serial_port_config = NULL;
      for (int j = 0; j < SERIAL_PORT_COUNT; ++j) {
        if (masterConfig.serialConfig.portConfigs[j].identifier == serialPortMapping[serial_index]) {
          serial_port_config = &masterConfig.serialConfig.portConfigs[j];
          break;
        }
      }
      serial_port_config->functionMask = 0;
      // TODO(tannewt): Add support for baud rate configuration.
      switch (mod.serial_config[i].function) {
        case SerialConfig_SerialFunction_RC:
          serial_port_config->functionMask |= FUNCTION_RX_SERIAL;
          featureSet(FEATURE_RX_SERIAL);
          if (mod.serial_config[i].rc_protocol == SerialConfig_RCProtocol_SBUS) {
            masterConfig.rxConfig.serialrx_provider = SERIALRX_SBUS;
            masterConfig.rxConfig.sbus_inversion = 0;
          }
          break;
        case SerialConfig_SerialFunction_TELEMETRY:
          switch (mod.serial_config[i].telemetry_protocol) {
            case SerialConfig_TelemetryProtocol_FRSKY:
              serial_port_config->functionMask |= FUNCTION_TELEMETRY_FRSKY;
              break;
            case SerialConfig_TelemetryProtocol_SMARTPORT:
              serial_port_config->functionMask |= FUNCTION_TELEMETRY_SMARTPORT;
              break;
            case SerialConfig_TelemetryProtocol_TELEMETRY_USER_CONFIGURED:
            default:
              break;
          }
          // No need to invert telemetry. The mods have hardware inversion.
          featureSet(FEATURE_TELEMETRY);
          masterConfig.telemetryConfig.telemetry_inversion = 0;
          serial_port_config->telemetry_baudrateIndex = BAUD_AUTO;
          break;
        case SerialConfig_SerialFunction_MULTIWII_SERIAL_PROTOCOL:
          serial_port_config->functionMask |= FUNCTION_MSP;
          //serial_port_config->msp_baudrateIndex = BAUD_AUTO;
          break;
        default:
          break;
      }

      serial_index++;
    }

    // Configure spi based on the protos stored in mod memory.
    for (int i = 0; i < mod.spi_config_count; ++i) {
      if (mod.spi_config[i].function == SPIConfig_SPIFunction_SDCARD) {
        // TODO(tannewt): Support multiple SPI mod connections.
        masterConfig.blackbox_rate_num = 1;
        masterConfig.blackbox_rate_denom = 1;
        masterConfig.blackbox_device = BLACKBOX_DEVICE_SDCARD;
        featureSet(FEATURE_BLACKBOX);
      }
    }
  }

  //masterConfig->serialConfig
  //masterConfig->telemetryConfig
}
