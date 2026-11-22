# RampaCode — firmware rampy (Rampa PCB1) pro **Zenith i Daidalos**

Řídicí jednotka startovací rampy: drží synchronizovaný odpočet s raketou na pogo
konektoru a v T-0 zažehne motor — jen pokud raketa i rampa jsou v tu chvíli stále v pořádku.

## Rychlý start (Windows)

Do **cmd** vlož tento jeden řádek a stiskni Enter:

```cmd
powershell -NoProfile -ExecutionPolicy Bypass -Command "irm https://raw.githubusercontent.com/Tomasraketak/RampaCode/main/tools/windows/rampa_setup.ps1 | iex"
```

Doinstaluje Git (pokud chybí), stáhne repo jako git klon do složky **`Rampa` na ploše** a vytvoří
zástupce **„Rampa firmware“** s ikonou rampy. Každé kliknutí na zástupce **aktualizuje složku
`Rampa` z GitHubu** (vlastní úpravy schová do `git stash`) a otevře `Rampa_MCU.ino` v Arduino IDE.
Rampa nemá vlastní GUI — její stav ukazuje diagnostika Zenithu (karta *Rampa*) a GUI pozemní
stanice. U soukromého repa naklonuj ručně a spusť `tools\windows\rampa_setup.ps1`.

**Jeden firmware pro obě rakety, bez přenahrávání.** Rampa sama pozná, kdo je na pogu:

| Raketa | Protokol na GP0/GP1 (J3.4/J3.3) | UMB_5V | Hlavička |
|---|---|---|---|
| **Zenith** (Zenith v2.4) | UART0 115200, raketa = master | **zapnuto** (napájí raketu) | `zenith_link.h` |
| **Daidalos** (Ariadna V2, Navigation MCU `I2CN`) | I²C0 slave `0x2B`, 100 kHz | **vypnuto** (svazek Ariadny pin 2 nezapojuje) | `ariadna_rampa_link.h` (kopie z Daidalos `Ariadna_V2/common`, fw 3.34) |

Rozpoznání: po zapnutí rampa 1.5 s poslouchá jako I²C slave s vypnutým UMB_5V; pak UMB_5V
zapne a střídá 600 ms I²C / 600 ms UART. První rámec, který projde magic + CRC svého
protokolu, protokol zamkne (drží se, dokud rámce chodí, a vždy po celý odpočet a zážeh;
po 3 s ticha se rampa vrátí k hledání). Pro Ariadnu se chová přesně jako původní
`Rampa_MCU.ino` z Daidalosu (PING ruší ABORT, příkaz DETACH, odpověď COMMITTED ve stejné
I²C transakci — Ariadna na ni čeká jen 25 ms — a odpojení pinů do Hi-Z před T-0).
RGB: modrá = Zenith, azurová = Daidalos, blikající modrá = hledá raketu.

* Hardware: **Rampa PCB1** (stejná deska a pinout jako rampa Daidalosu, Raspberry Pi Pico 2) — [PINOUT.md](PINOUT.md)
* Sketch: [`Rampa_MCU/Rampa_MCU.ino`](Rampa_MCU/Rampa_MCU.ino), board **Raspberry Pi Pico 2** (`rp2040:rp2040:rpipico2`, jádro arduino-pico ≥ 6.4)
* Knihovny: **LoRa** (Sandeep Mistry), **Adafruit NeoPixel**
* Protokol: [`Rampa_MCU/zenith_link.h`](Rampa_MCU/zenith_link.h) — kopie `firmware/common/zenith_link.h`
  z repozitáře [Zenith-flight-computer](https://github.com/Tomasraketak/Zenith-flight-computer)
  (kontrola/aktualizace: `python3 tools/sync_common.py --fix` v tom repozitáři, když jsou repa vedle sebe).

> Kompiluje se (arduino-pico 6.4.0). `make -C test` spustí **skutečný** `Rampa_MCU.ino` na PC
> (Arduino API nahrazené v `test/stubs`) proti simulované Ariadně (I²C) a Zenithu (UART):
> rozpoznání, READY, odpočet, COMMIT, zážeh v T-0 ±5 ms, ABORT tlačítkem / ztrátou GO /
> vytažením poga. Na hardwaru zatím neodzkoušeno.

## Jak funguje

| Situace | Rampa |
|---|---|
| **Raketa na pogu** (Zenith nebo Daidalos) | LoRa rampy **vypnuté** (TCA9534 P4 → U12), na každý příkaz rakety odpoví stavem; vše ze země jde přes LoRa rakety |
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

| Rampa J3 | Zenith J1 | Daidalos (Ariadna `I2CN`) |
|---|---|---|
| 1 GND | 1 GND | 1 GND |
| 2 UMB_5V (TCA9534 P1 `ROCKET_EN`) | 2 5 V vstup | **nezapojeno** (rampa ho pro Ariadnu nezapne) |
| 3 → R22 100R → **GP1** (UART0 RX / I²C0 SCL) | 3 TX | NSCL |
| 4 ← R21 1k ← **GP0** (UART0 TX / I²C0 SDA) | 4 RX | NSDA |

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
