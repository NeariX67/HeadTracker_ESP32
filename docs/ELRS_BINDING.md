# ELRS Backpack Binding Support

This document describes the ExpressLRS (ELRS) Backpack binding protocol support added to HeadTracker_ESP32.

## Overview

The HeadTracker_ESP32 now supports binding with ExpressLRS Backpack devices using the MSP (MultiWii Serial Protocol) over ESP-NOW. This allows the headtracker to work seamlessly with ELRS VRx backpack modules.

## Protocol Details

### MSP V2 Packet Structure

ELRS Backpack uses MSP V2 protocol for communication:

```
Offset  Usage           In CRC  Comment
======  ======          ======  ========
0       $                       Framing magic start char
1       X                       'X' in place of v1 'M'
2       type                    '<' / '>' Message Type (Command/Response)
3       flag            +       uint8, flag (set to zero)
4       function        +       uint16 (little endian) - Function code
6       payload size    +       uint16 (little endian) - Payload size in bytes
8       payload         +       n bytes payload
n+8     checksum                uint8, crc8_dvb_s2 checksum
```

### Binding Process

1. **Enter Binding Mode**: The receiver enters binding mode by holding the bind button or through power cycling.

2. **Broadcast Binding Message**: The device sends broadcast messages on the ESP-NOW channel.

3. **MSP_ELRS_BIND Command**: When an ELRS Backpack device wants to bind, it sends an MSP packet with:
   - Function code: `0x09` (MSP_ELRS_BIND)
   - Payload: 6-byte MAC address of the transmitter

4. **MAC Address Extraction**: The receiver extracts the MAC address from the MSP payload (not from the ESP-NOW sender address).

5. **Peer Addition**: The extracted MAC address is added as a peer for future communication.

6. **Persistent Storage**: The binding information is saved to NVS (Non-Volatile Storage).

### Backward Compatibility

The implementation maintains full backward compatibility with the legacy binding protocol:

- **Legacy Protocol**: Uses a custom frame structure with "TXTXTX" payload and CRC-8 validation
- **ELRS Protocol**: Uses MSP V2 packets with MSP_ELRS_BIND command

The binding task automatically detects which protocol is being used and handles both seamlessly.

## Implementation

### Files Modified

1. **include/app_espnow.h**
   - Added MSP packet structure definitions
   - Added MSP state machine enums
   - Added `MSP_ELRS_BIND` constant (0x09)

2. **src/app_espnow.c** (ESP-IDF version)
   - Implemented MSP packet parser
   - Implemented CRC8 DVB-S2 checksum
   - Updated binding task to support both protocols

3. **src/app_espnow.cpp** (Arduino version)
   - Same MSP implementation for ESP8266 compatibility

### Key Functions

- `crc8_dvb_s2_byte()`: Calculates CRC8 DVB-S2 checksum for MSP packets
- `msp_process_byte()`: State machine for parsing MSP packets byte-by-byte
- `msp_parse_buffer()`: Parses a complete buffer for MSP packets
- `espnow_bind_task()`: Handles binding for both legacy and ELRS protocols

## Testing

To test ELRS Backpack binding:

1. Flash the updated firmware to your HeadTracker device
2. Enter binding mode on the HeadTracker
3. Send an MSP_ELRS_BIND packet from an ELRS Backpack device
4. The devices should bind and the MAC address will be stored

## Reference

This implementation is based on the ExpressLRS Backpack VRx code:
https://github.com/ExpressLRS/Backpack/blob/b4745f55f8f317b3b98990b10499521cc9ada1ee/src/Vrx_main.cpp#L165

## Future Enhancements

Potential future improvements:
- Support for additional MSP commands (VTX config, head tracking control, etc.)
- Two-way communication with ELRS Backpack devices
- Integration with ELRS telemetry
