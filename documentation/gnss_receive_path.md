# GNSS receive path (USART3)

`Communication/GNSS_driver.cpp` receives the GNSS data with a circular DMA
buffer and feeds it byte by byte to `Drivers/Custom/UBX_receiver`:

- It pairs NAV-PVT with the heading message of the same iTOW: NAV-RELPOSNED
  (F9P) or NAV-DAHEADING (X20D). The order doesn't matter; the X20D sends
  NAV-DAHEADING about 7 ms before NAV-PVT.
- Other UBX messages and NMEA are skipped, so extra output of the receiver
  doesn't break the reception.
- A PVT whose heading message doesn't come is passed on alone once the line
  has been idle for 20 ms. The epoch then has no heading (`SAT_HEADING`
  cleared).

`GNSS_type::update_combined()` evaluates the heading message before the
PVT, because `update()` signals new data to the communicator task, which
runs at a higher priority.

`sw_stm32/tests/UBX_receiver_test.cpp` checks the receiver on the host with
recorded receiver output (`sw_stm32/tests/data`).
