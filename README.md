# RampaCode — firmware rampy (Rampa PCB1) pro raketu Zenith

Řídicí jednotka startovací rampy: napájí raketu po pogo konektoru, mluví s ní po UARTu,
drží synchronizovaný odpočet a v T-0 zažehne motor — jen pokud raketa i rampa jsou
v tu chvíli stále v pořádku.

* Hardware: **Rampa PCB1** (stejná deska a pinout jako rampa Daidalosu, Raspberry Pi Pico 2) — [PINOUT.md](PINOUT.md)
* Sketch: [`Rampa_MCU/Rampa_MCU.ino`](Rampa_MCU/Rampa_MCU.ino), board **Raspberry Pi Pico 2** (`rp2040:rp2040:rpipico2`, jádro arduino-pico ≥ 6.4)
* Knihovny: **LoRa** (Sandeep Mistry), **Adafruit NeoPixel**
* Protokol: [`Rampa_MCU/zenith_link.h`](Rampa_MCU/zenith_link.h) — kopie `firmware/common/zenith_link.h`
  z repozitáře [Zenith-flight-computer](https://github.com/Tomasraketak/Zenith-flight-computer)
  (kontrola/aktualizace: `python3 tools/sync_common.py --fix` v tom repozitáři, když jsou repa vedle sebe).

> Kompiluje se (arduino-pico 6.4.0), na hardwaru zatím neodzkoušeno.

## Jak funguje

| Situace | Rampa |
|---|---|
| **Raketa na pogu** (rámce od rakety < 1.5 s) | LoRa rampy **vypnuté** (TCA9534 P4 → U12), na každý příkaz rakety odpoví stavem; vše ze země jde přes LoRa rakety |
| **Bez rakety** | po 2 s zapne LoRa a 1× za s vysílá beacon (baterie, klíč, kontinuita, proud do poga); ve stejných 10 s oknech jako raketa přijímá: napájení rakety ZAP/VYP, ABORT, SAFE, bzučák |
| **Po zážehu** | 30 min nechává rádio vypnuté (pásmo patří letící raketě) |

### Odpočet a zážeh

1. Raketa posílá `PREPARE` → rampa zkontroluje klíč (J4), rozbušku na zvoleném kanálu,
   baterii, tlačítko a I²C periférie → `READY` (jinak `FAULT` s maskou chyb).
2. `COUNTDOWN` každých 50 ms s ms do T-0 → rampa si T-0 zamkne (skok > 200 ms = chyba
   časování → ABORT) a pořád kontroluje, že raketa hlásí **GO**.
3. `COMMIT` v posledních 300 ms → `COMMITTED`; od teď rampa zažehne i bez dalších zpráv.
4. **T-0:** poslední kontrola (klíč, tlačítko, baterie, kontinuita) → `PYRO_ARM` + kanál
   na `rampa_fire_ms` (z rakety, výchozí 1 s) → `FIRED`.

ABORT vyvolá: příkaz rakety, raketa přestane hlásit GO, raketa mlčí > 1.5 s během odpočtu,
lokální tlačítko SW1, jakákoli chyba blokování, chybějící COMMIT. ABORT je zablokovaný
(latched) do příkazu `SAFE`.

### Pogo umbilical

| Rampa J3 | | Zenith J1 |
|---|---|---|
| 1 GND | — | 1 GND |
| 2 UMB_5V (TCA9534 P1 `ROCKET_EN`) | → | 2 5V |
| 3 → R22 100R → **GP1 (UART0 RX)** | ← | 3 TX |
| 4 ← R21 1k ← **GP0 (UART0 TX)** | → | 4 RX |

UART 115200 8N1, rámce `0x5A 0xE7 TYPE LEN payload CRC16`; raketa = master
(`ZL_PKT_RMP_CMD` 12 B), rampa odpovídá `ZL_PKT_RMP_STATUS` 20 B.

### Signalizace

| RGB / světlo / bzučák | Význam |
|---|---|
| modrá (svítí) / modrá blikání | IDLE s raketou / bez rakety |
| zelená | READY |
| žlutá bliká | FAULT (viz maska chyb na konzoli) |
| oranžová rychle + pípání | odpočet (posledních 10 s rychleji) |
| červená + červené světlo + bzučák | COMMIT / zážeh |
| fialová | ABORT |

USB konzole (115200) vypisuje 1× za sekundu stav, kontinuitu, klíč a proud do rakety.
