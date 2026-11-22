# Rampa PCB1 — pinout (RampaNetlist_PCB1_2026-11-19)

Stejná deska jako rampa Daidalosu. Pogo umbilical (J3) je podle připojené rakety buď
**UART0** (Zenith: GP0 = TX, GP1 = RX) nebo **I²C0 slave 0x2B** (Daidalos/Ariadna: GP0 = SDA,
GP1 = SCL) — RP2350 má na těchto pinech obojí a firmware přepíná sám.

```yaml
rampa_pcb1: # Raspberry Pi Pico 2 (U7), sketch Rampa_MCU/Rampa_MCU.ino
  umbilical_J3:    {gp0: "R21 1k -> J3.4", gp1: "R22 100R <- J3.3 (R23 100k PU)",
                    zenith:  {mode: "UART0 115200", gp0: TX, gp1: RX, umb_5v: on},
                    ariadna: {mode: "I2C0 slave 0x2B 100 kHz", gp0: SDA, gp1: SCL, umb_5v: off, detach: "Hi-Z after COMMITTED"},
                    detect: "1.5 s I2C (UMB_5V off), then 600 ms I2C / 600 ms UART until a valid frame locks"}
  i2c1_internal:   {sda: GP6, scl: GP7, clk_khz: 400, pullup: "R18/R19 4.7k",
                    devs: {tca9534: 0x20, ina226_vbat: 0x40, ina226_umb5v: 0x41, ads1115: 0x48}}
  tca9534_0x20:    {P0: "BUZZER_EN out H", P1: "ROCKET_EN out H (UMB_5V buck U2 = napajeni rakety)",
                    P2: "KEY (RBF_SENSE) in H = klic J4 sepnut", P3: "LORA_RST out L = reset",
                    P4: "LORA_EN out H (U12 TPS22918; LOW kdyz je raketa na pogu)", P5: "LORA_DIO0 in (polled, nepouzito)",
                    P6: "FIRE_LED out H", P7: "BTN SW1 in L = lokalni ABORT"}
  ads1115_0x48:    {AIN0: CONT1, AIN1: CONT2, AIN2: CONT3, AIN3: "5V_SENS/2", threshold_mv: 350}
  pyro:            {arm: "GP19 -> Q21 -> Q20 AO4407A (VPYRO_ARMED)", ch1: "GP16 -> Q22 -> J5", ch2: "GP17 -> Q23 -> J6",
                    ch3: "GP18 -> Q24 -> J7", key: "J4 + F2 5A v serii s VBAT -> VPYRO_KEY"}
  lora:            {spi1: {sck: GP10, mosi: GP11, miso: GP12, cs: GP13}, ic: "M1 Ra-02 SX1278 433 MHz",
                    rst: "TCA9534 P3", power: "TCA9534 P4"}
  rgb:             "GP20 SK6812 (R56)"
  unused_by_fw:    {spi0_flash_ads1220: "GP2-GP5, GP15, GP21", servos: "GP8 GP9 GP14 GP22", pressure_adc: GP26, umb2: "GP27 GP28"}
```

| Pin | Funkce | Síť / periférie |
|---|---|---|
| GP0 | UART0 TX (Zenith) / I²C0 SDA (Daidalos) | `UMB_TX` → R21 1 kΩ → J3.4 |
| GP1 | UART0 RX (Zenith) / I²C0 SCL (Daidalos) | `UMB_RX` ← R22 100 Ω ← J3.3 (R23 100 kΩ PU) |
| GP6 / GP7 | I2C1 SDA / SCL | TCA9534 `0x20`, INA226 `0x40` (VBAT), INA226 `0x41` (UMB_5V), ADS1115 `0x48` |
| GP10–GP13 | SPI1 SCK/MOSI/MISO/CS | Ra-02 LoRa (napájení přes U12) |
| GP16 / GP17 / GP18 | zážeh kanálu 1 / 2 / 3 | Q22 / Q23 / Q24 → J5 / J6 / J7 |
| GP19 | ARM | Q21 → Q20 (VPYRO_ARMED) |
| GP20 | RGB | SK6812 |

Zážeh vyžaduje současně: klíč J4 sepnutý + GP19 HIGH + GPx HIGH.
