# Recorded GNSS receiver output

Raw UBX streams for `UBX_receiver_test.cpp`, 30 s each, recorded over USB at
10 Hz with the settings in the receiver's RAM only.

- `<name>.ubx`: the bytes as read from the receiver.
- `<name>.csv`: `t_s,offset,length` per read, `t_s` in s since the start.
  USB delivers in batches, so the times are accurate to a few ms only.

| File | Receiver | Messages |
|---|---|---|
| `f9p_hpg151_baseboard-outputs_10hz_usb_30s` | ZED-F9P, HPG 1.51, no base | as `configuration/Ardusimple_Heading_Baseboard_100ms.txt` sets on UART1: NAV-PVT, NAV-RELPOSNED, NAV-TIMEGPS, MON-HW, MON-HW2 |
| `f9p_hpg151_pvt-relposned_10hz_usb_30s` | ZED-F9P, HPG 1.51, no base | NAV-PVT, NAV-RELPOSNED |
| `f9p_hpg151_sapos-rtk-fixed_pvt-relposned_10hz_usb_30s` | ZED-F9P, HPG 1.51, RTCM from SAPOS Bayern (VRS), RTK fixed | NAV-PVT, NAV-RELPOSNED (valid, fixed), RXM-RTCM |
| `x20d_hdg200_pvt-daheading_10hz_usb_30s` | ZED-X20D, HDG 2.00, two antennas | NAV-PVT, NAV-DAHEADING as `configuration/larus_config_X20D.ucf` sets on UART1 |

Without a base, the F9P sends NAV-RELPOSNED only now and then. The X20D
sends NAV-DAHEADING about 7 ms before the NAV-PVT of the same epoch; its
heading was not valid during the recording (indoor antennas).

With RTK fixed, the F9P still sends NAV-RELPOSNED only in short bursts (22 in
this excerpt) and skips some epochs at 10 Hz (iTOW steps of 200 ms).
