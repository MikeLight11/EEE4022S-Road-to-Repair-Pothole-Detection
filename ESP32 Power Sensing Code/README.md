# Power Controller

ESP32-C3 SuperMini firmware that monitors a 12 V supply rail (valid range 9–14 V)
through a resistive divider on an ADC pin and logs the reconstructed voltage over
the USB serial console once per second.

## Hardware

```
12V rail ---[ R1 = 100k ]---+---[ R2 = 20k ]--- GND
                            |
                            +---+--- GPIO3 (ADC1_CH3)
                                |
                              100nF
                                |
                               GND
```

- `Vadc = Vin * R2 / (R1 + R2) = Vin / 6`
  - 9.0 V in  -> 1.50 V at pin
  - 14.0 V in -> 2.33 V at pin
- ADC1, 12 dB attenuation (~0–3.1 V full scale at the pin), 12-bit.
- **Share grounds:** the 12 V supply ground and the ESP32-C3 ground must be common.
- The 100 nF cap from GPIO3 to GND filters switching noise; optional but recommended.

If you change `R1`/`R2`, update `DIVIDER_R1_OHMS` / `DIVIDER_R2_OHMS` in
[main/power_controller.c](main/power_controller.c). Keep the pin voltage below
~3.0 V at the maximum expected `Vin`.

## Build / flash / monitor

```sh
source ~/.espressif/tools/activate_idf_v6.1.sh
idf.py set-target esp32c3        # already done; sdkconfig committed-out via .gitignore
idf.py build
idf.py -p /dev/tty.usbmodemXXXX flash monitor
```

Exit the monitor with `Ctrl+]`.

## Expected output

```
I (315) power_ctrl: Power Controller starting
I (315) power_ctrl: Divider scale = 6.000 (R1=100000, R2=20000)
I (325) power_ctrl: ADC calibration enabled
I (1330) power_ctrl: ADC pin = 2010 mV  |  Vin = 12.06 V  |  OK
I (2330) power_ctrl: ADC pin = 1495 mV  |  Vin = 8.97 V  |  UNDERVOLTAGE
```

## How it works

- `adc_oneshot` reads GPIO3, averaging 64 samples per reading.
- `adc_cali` (curve-fitting on the C3) converts the raw count to millivolts using
  the chip's factory eFuse calibration. If calibration is unavailable it falls
  back to a linear `raw * 3100 / 4095` estimate.
- The pin voltage is multiplied by the divider scale to recover `Vin`, which is
  logged with an `OK` / `UNDERVOLTAGE` / `OVERVOLTAGE` flag against the 9–14 V window.
