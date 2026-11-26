# RampaCode — firmware rampy (Rampa PCB1) pro **Zenith i Daidalos**

Řídicí jednotka startovací rampy: drží synchronizovaný odpočet s raketou na pogo
konektoru a v T-0 zažehne motor — jen pokud raketa i rampa jsou v tu chvíli stále v pořádku.

## Rychlý start (Windows)

Do **cmd** vlož tento jeden řádek a stiskni Enter:

```cmd
curl.exe -fsSL https://raw.githubusercontent.com/Tomasraketak/RampaCode/main/tools/windows/rampa_setup.cmd -o "%TEMP%\rampa_setup.cmd" && "%TEMP%\rampa_setup.cmd"
```

Doinstaluje Git a Python 3.12 (pokud chybí), stáhne repo jako git klon do složky **`Rampa` na ploše**
a vytvoří dva zástupce s ikonou rampy:

* **„Rampa diagnostika“** — aktualizuje složku `Rampa` z GitHubu (vlastní úpravy schová do `git stash`)
  a spustí samostatnou USB diagnostiku rampy ([`gui/rampa_diag.py`](gui/rampa_diag.py), vzhled a ovládání
  ve stylu diagnostiky Daidalosu a Zenithu). Stačí připojit rampu přímo k PC přes USB — bez nutnosti
  propojovat ji se Zenithem nebo pozemní stanicí (obsahuje i režim **Simulace** bez hardwaru).
* **„Rampa firmware“** — aktualizuje složku `Rampa` a otevře [`Rampa_MCU/Rampa_MCU.ino`](Rampa_MCU/Rampa_MCU.ino)
  v Arduino IDE (dosavadní postup zůstává zachován).

Dosavadní způsoby diagnostiky (přes kartu *Rampa* v diagnostice Zenithu, přes GUI pozemní stanice i
1 Hz výpis na USB Serial Monitoru 115200 Bd) jsou plně zachovány beze změny. Popis USB příkazů a
telemetrie je v [`docs/USB_PROTOCOL.md`](docs/USB_PROTOCOL.md). U soukromého repa naklonuj ručně a
spusť `tools\windows\rampa_setup.cmd`.

Spuštění diagnostiky ručně (Windows / Linux / macOS):

```bash
pip install -r gui/requirements.txt
python3 gui/rampa_diag.py          # nebo s --demo pro simulaci
```

**Jeden firmware pro obě rakety i oba konektory pupeční šňůry (`J3` i `J14`), bez přenahrávání.** Rampa sama pozná, jaká raketa a na kterém konektoru je připojená:

| Raketa | Protokol na `J3` (GP0/GP1) nebo `J14` (GP8/GP9) | UMB_5V | Hlavička |
|---|---|---|---|
| **Zenith** (Zenith v2.4) | UART0 (`J3`) / UART1 (`J14`) 115200, raketa = master | **zapnuto** (napájí raketu) | `zenith_link.h` |
| **Daidalos** (Ariadna V2, Navigation MCU `I2CN`) | I²C0 slave `0x2B`, 100 kHz (`J14` má 4.7 kΩ pull-upy, funguje i `J3`) | **vypnuto** (svazek Ariadny pin 2 nezapojuje) | `ariadna_rampa_link.h` (kopie z Daidalos `Ariadna_V2/common`, fw 3.34) |

Rozpoznání: po zapnutí rampa 1.5 s poslouchá s vypnutým `UMB_5V` (750 ms `J14` I²C0 + `J3` UART0,
pak 750 ms `J3` I²C0 + `J14` UART1); pak `UMB_5V` zapne a střídá oba sloty po 600 ms. První rámec,
který projde magic + CRC svého protokolu, zamkne protokol i konkrétní konektor (`J3` nebo `J14`)
bez přerušení komunikace (drží se, dokud rámce chodí, a vždy po celý odpočet a zážeh; po 3 s ticha
se rampa vrátí k hledání). Pro Ariadnu se chová přesně jako původní `Rampa_MCU.ino` z Daidalosu
(PING ruší ABORT, příkaz DETACH, odpověď COMMITTED ve stejné I²C transakci — Ariadna na ni čeká jen
25 ms — a odpojení pinů do Hi-Z před T-0).
RGB: modrá = Zenith, azurová = Daidalos, blikající modrá = hledá raketu.

* Hardware: **Rampa PCB1** (stejná deska a pinout jako rampa Daidalosu, Raspberry Pi Pico 2) — [PINOUT.md](PINOUT.md), kompletní netlist: [`hardware/RampaPCBnet.enet`](hardware/RampaPCBnet.enet)
* Sketch: [`Rampa_MCU/Rampa_MCU.ino`](Rampa_MCU/Rampa_MCU.ino), board **Raspberry Pi Pico 2** (`rp2040:rp2040:rpipico2`, jádro arduino-pico ≥ 6.4)
* Knihovny: **LoRa** (Sandeep Mistry), **Adafruit NeoPixel**
* Protokol: [`Rampa_MCU/zenith_link.h`](Rampa_MCU/zenith_link.h) — kopie `firmware/common/zenith_link.h`
  z repozitáře [Zenith-flight-computer](https://github.com/Tomasraketak/Zenith-flight-computer)
  (kontrola/aktualizace: `python3 tools/sync_common.py --fix` v tom repozitáři, když jsou repa vedle sebe).

> Kompiluje se (arduino-pico 6.4.0). `make -C test` spustí **skutečný** `Rampa_MCU.ino` na PC
> (Arduino API nahrazené v `test/stubs`) proti simulované Ariadně (I²C na `J14` i `J3`) a Zenithu
> (UART na `J3` i `J14`): rozpoznání, READY, odpočet, COMMIT, zážeh v T-0 ±5 ms, ABORT tlačítkem /
> ztrátou GO / vytažením poga i samostatnou USB diagnostiku. Na hardwaru zatím neodzkoušeno.

## Jak funguje

| Situace | Rampa |
|---|---|
| **Raketa na pogu** (Zenith nebo Daidalos, `J3` nebo `J14`) | LoRa rampy **vypnuté** (TCA9534 P4 → U12), na každý příkaz rakety odpoví stavem; vše ze země jde přes LoRa rakety |
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

### Pogo umbilical (`J3` a `J14`)

Oba 4pinové konektory mají stejné rozložení pinů a sdílenou větev `UMB_5V`:

| Pin | `J3` (GP0/GP1) | `J14` (GP8/GP9, 4.7 kΩ PU) | Zenith `J1` | Daidalos (Ariadna `I2CN`) |
|---|---|---|---|---|
| 1 | GND | GND | 1 GND | 1 GND |
| 2 | UMB_5V (TCA9534 P1 `ROCKET_EN`) | UMB_5V (společná větev) | 2 5 V vstup | **nezapojeno** (rampa ho pro Ariadnu nezapne) |
| 3 | → R22 100R → **GP1** (UART0 RX / I²C0 SCL) | → R10 100R → **GP9** (UART1 RX / I²C0 SCL) | 3 TX | NSCL |
| 4 | ← R21 1k ← **GP0** (UART0 TX / I²C0 SDA) | ← R9 100R ← **GP8** (UART1 TX / I²C0 SDA) | 4 RX | NSDA |

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
