# GL-S10 ESPHome Wi-Fi, BLE proxy, and A2DP audio

This is a complete, device-specific ESPHome example for a GL.iNet GL-S10. It uses
Wi-Fi for the ESPHome native API and OTA updates, forwards BLE advertisements
and active BLE connections, and sends stereo 44.1 kHz audio over Bluetooth
Classic A2DP. Three independent media players can retain separate receiver
selections. Only one A2DP connection plays at a time; starting another player
preempts the current one and changes its state to idle. The connection closes
after playback stops.

The example exposes Classic discovery, bond lists, pairing, selection,
confirmation, unpairing, and forgetting through native API actions and status
entities. AVRCP absolute volume controls the receiver. The announcement
pipeline accepts MP3 and the resampler supplies 16-bit stereo PCM to A2DP.
Counters for BLE advertisements, audio PCM bytes, underruns, and free memory
support direct checks of coexistence and audio delivery.

## Code and hardware

Build with the ESPHome source from
[`mbrudka/esphome` commit `61ba90d47`](https://github.com/mbrudka/esphome/commit/61ba90d47fb10d4cc228ccb5ee342268de96ba74),
which contains the stacked Classic controller, single-receiver A2DP, and
receiver-management changes. These changes are under review and are not part of
the published ESPHome release yet. The corresponding code reviews are
[`esphome/esphome#21408`](https://github.com/esphome/esphome/pull/21408),
[`mbrudka/esphome#2`](https://github.com/mbrudka/esphome/pull/2), and
[`mbrudka/esphome#1`](https://github.com/mbrudka/esphome/pull/1).

The example follows the audio and BLE configuration tested on one GL-S10,
using an original ESP32 with ESP-IDF. This public version passed ESPHome
configuration validation and a complete firmware build against the linked
commit; it has not itself been flashed onto hardware. The 4 MB build leaves
about 2% of the application partition free. GL-S10 board revisions differ;
check the revision and available PSRAM before using other hardware. The official
[GL-S10 flashing guide](https://blakadder.com/gl-s10/) shows how to open the
case and connect a USB-to-UART adapter for the first flash. Its ready-made
installer writes a different Ethernet BLE-proxy firmware. This example's
firmware must be built from the source above.

## Setup

1. Copy `secrets.example.yaml` to `secrets.yaml`. Replace all example values.
   Generate the API key with `openssl rand -base64 32` and use an independent
   random password for the fallback access point. OTA inherits the API key
   and encrypts firmware uploads. Do not publish `secrets.yaml`.
2. Build the configuration with the linked ESPHome commit:
   `esphome config gl-s10-a2dp-wifi.yaml`, then
   `esphome compile gl-s10-a2dp-wifi.yaml`.
3. For first installation, follow the GL-S10 USB-to-UART wiring guide and flash
   the compiled image by serial. Subsequent updates can use OTA while Wi-Fi
   remains reachable. Keep a known-good image for recovery.
4. Start `a2dp_start_discovery`, read `Classic Discovery Page` and `Classic
   Discovery Count`, then cancel discovery before pairing. Use `a2dp_pair` with
   a slot number (1–3), the receiver's Bluetooth Classic address, and its PIN
   or an empty PIN if no PIN is requested. `a2dp_pair_incoming` handles a
   receiver that initiates the bond. Watch the pairing-status and confirmation
   entities; use `a2dp_confirm` for numeric comparison.
5. Play an MP3 through one of the three media players at a low initial volume.
   To change a slot's receiver, use `a2dp_select` with a saved bond. The
   actions `a2dp_clear_selection`, `a2dp_unpair`, and `a2dp_forget_address`
   remove a selection, the slot's bond, or a bond by address respectively.

The action names above are ESPHome native API actions. A client can call them
directly; Home Assistant is optional for operating the device. Discovery and
bond lists are paged at ten devices per page. Use the page-selection actions
to inspect additional pages.

## Operating limits

Wi-Fi, Bluetooth Classic and BLE share the ESP32 radio. Connection setup and
audio playback can delay BLE reports or Wi-Fi packets. A receiver must be on,
reachable, and accepting a connection. Classic page timeouts may require a
retry; `preconnect_timeout: 60s` allows the bounded connection retries to
finish before the media player downloads audio. The successful test on one
GL-S10 does not establish reliability for every receiver or board revision.

The sample has no receiver address preconfigured. Pair or select a receiver
before playback. The side button is reported as a binary sensor; pressing it
does not physically reset the ESP32.
