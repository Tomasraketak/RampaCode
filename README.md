# RampaCode — firmware rampy (Rampa PCB1) pro **Zenith, Daidalos i test motoru**

Řídicí jednotka startovací rampy: drží synchronizovaný odpočet s raketou na pogo
konektoru a v T-0 zažehne motor — jen pokud raketa i rampa jsou v tu chvíli stále v pořádku.

**Bez rakety je z rampy zkušební stav motoru:** zapálí motor povelem z pozemní stanice, měří tah
(tenzometr na ADS1220), tlak v komoře (čidlo 0,5–4,5 V / 0–500 psi) a dvě teploty (termistory),
zaznamenává vše **2000× za sekundu** na flash W25Q128 a živá data posílá vlastní LoRou na pozemní
stanici → [Test motoru](#test-motoru-měření-tahu). Diagnostické GUI rampy (USB): [`gui/rampa_diag.py`](gui/rampa_diag.py).

## Rychlý start (Windows)

Do **cmd** vlož tento jeden řádek a stiskni Enter:

```cmd
powershell -NoProfile -ExecutionPolicy Bypass -Command "irm https://raw.githubusercontent.com/Tomasraketak/RampaCode/main/tools/windows/rampa_setup.ps1 | iex"
```

Doinstaluje Git a Python 3.12 (pokud chybí), stáhne repo jako git klon do složky **`Rampa` na ploše**
a vytvoří dva zástupce s ikonou rampy. Každé kliknutí **aktualizuje složku `Rampa` z GitHubu**
(vlastní úpravy schová do `git stash`) a pak:

* **„Rampa firmware“** otevře `Rampa_MCU.ino` v Arduino IDE,
* **„Rampa diagnostika“** aktualizuje Python balíčky a spustí diagnostické GUI rampy (USB).

U soukromého repa naklonuj ručně a spusť `tools\windows\rampa_setup.ps1`.

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

* Firmware: **3.00** (Zenith + Daidalos + test motoru)
* Hardware: **Rampa PCB1** (stejná deska a pinout jako rampa Daidalosu, Raspberry Pi Pico 2) — [PINOUT.md](PINOUT.md), kompletní netlist: [`hardware/RampaPCBnet.enet`](hardware/RampaPCBnet.enet)
* Sketch: [`Rampa_MCU/Rampa_MCU.ino`](Rampa_MCU/Rampa_MCU.ino), board **Raspberry Pi Pico 2** (`rp2040:rp2040:rpipico2`, jádro arduino-pico ≥ 6.4)
* Knihovny: **LoRa** (Sandeep Mistry), **Adafruit NeoPixel** (SPI, ADC a druhé jádro jsou z jádra arduino-pico)
* Měření a záznam: [`Rampa_MCU/thrust_stand.h`](Rampa_MCU/thrust_stand.h) (jádro 1), protokol se stanicí
  [`Rampa_MCU/stand_link.h`](Rampa_MCU/stand_link.h) (kopie je v repu GroundStation), USB: [`docs/USB_PROTOCOL.md`](docs/USB_PROTOCOL.md)
* Protokol: [`Rampa_MCU/zenith_link.h`](Rampa_MCU/zenith_link.h) — kopie `firmware/common/zenith_link.h`
  z repozitáře [Zenith-flight-computer](https://github.com/Tomasraketak/Zenith-flight-computer)
  (kontrola/aktualizace: `python3 tools/sync_common.py --fix` v tom repozitáři, když jsou repa vedle sebe).

> `make -C test` spustí **skutečný** `Rampa_MCU.ino` na PC (Arduino API nahrazené v `test/stubs`,
> obě jádra střídavě) proti simulované Ariadně (I²C), Zenithu (UART), pozemní stanici (LoRa) a
> simulovanému ADS1220 + W25Q128 + tlakovému čidlu: rozpoznání, odpočet, zážeh v T-0, všechny ABORTy,
> a celý test motoru (ARM, blokování, FIRE, zážeh ±2 ms, záznam 2000 S/s, výsledky, křivka, DUMP,
> ztráta spojení, tlačítko, klíč, nezážeh, chybějící ADS1220). `QT_QPA_PLATFORM=offscreen python3
> test/gui_smoke.py` otestuje diagnostické GUI proti simulované rampě a dekodér záznamů proti
> skutečnému výpisu z firmwaru. **Verze 3.00 (test motoru) zatím nebyla přeložena pro Pico ani
> vyzkoušena na hardwaru** – před prvním ostrým zážehem projdi [zkoušky](#zkoušky-před-prvním-zážehem).

## Jak funguje

| Situace | Rampa |
|---|---|
| **Raketa na pogu** (Zenith nebo Daidalos) | LoRa rampy **vypnuté** (TCA9534 P4 → U12), na každý příkaz rakety odpoví stavem; vše ze země jde přes LoRa rakety |
| **Bez rakety** | po 2 s zapne LoRa a 1× za s vysílá beacon (baterie, klíč, kontinuita, proud do poga); přijímá (mimo vlastní vysílání pořád): napájení rakety ZAP/VYP, ABORT, SAFE, bzučák |
| **Bez rakety, stanice v režimu TEST MOTORU** | první paket stanice přepne rampu na zkušební stav motoru (pogo se zavře) – viz níže; po 60 s bez stanice se vrátí k hledání rakety |
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

USB (115200): stav `$S` 5×/s, živé měření `$M` 50×/s a události `# …` – pro diagnostické GUI, čitelné i v Serial Monitoru ([protokol](docs/USB_PROTOCOL.md)).

## Test motoru (měření tahu)

Raketa není připojená, rampa PCB slouží jako zapalovač a měřicí ústředna motorového stojanu.

### Zapojení

| Konektor | Co | Poznámka |
|---|---|---|
| **J8** (4 pin) | tenzometr: 1 = +exc (5V_SENS), 2 = GND (−exc), 3 = S+ (LC_SP), 4 = S− (LC_SN) | plný můstek, buzení 5 V z LP2985, ADS1220 PGA 128, ratiometricky |
| **J9** (3 pin) | tlakové čidlo 0,5–4,5 V: 1 = +5 V, 2 = signál, 3 = GND | dělič 4k7/10k na GP26, napájení měřené (ratiometrický přepočet) |
| **J10 / J11** (2 pin) | termistor 1 / 2 (NTC mezi 5V_SENS a vstupem, 10k k zemi na desce) | výchozí NTC 10k B3950, jinak nastavit v diagnostice |
| **J5 / J6 / J7** | rozbuška motoru (výstup 1 / 2 / 3) | volí se ve stanici, kontinuita se měří |
| **J4** | klíč (pyro napájení) | bez klíče zážeh hardwarově nemožný |

Pokud tah vychází záporně, přehoď S+ a S− (nebo nech kalibraci závažím – záporné měřítko vadí jen v grafu
nekalibrovaného signálu).

### Postup (pozemní stanice, režim **TEST MOTORU · tah**)

1. Rampu zapni **bez rakety na pogu**, stanici přepni na *TEST MOTORU*. Stanice rampu sama „probudí“ – do ~1 s
   začne chodit telemetrie (2 Hz, při testu 5 Hz s 50 Hz průběhem tahu).
2. Motor upevněný, rozbuška zatím nezapojená: **TARE** (nula tahu), případně **Nula tlaku**.
   Kalibrace závažím jde ze stanice i z diagnostiky (USB); uloží se v rampě.
3. Zkontroluj automatické kontroly (spojení, klíč, kontinuita zvolené rozbušky, baterie, tenzometr, flash),
   zapoj rozbušku, zapni klíč a odškrtni ruční bezpečnostní body.
4. **① ODJISTIT (ARM)** – rampa znovu ověří blokování a smaže na flash místo pro záznam (pár sekund),
   pak pípá a čeká. Po 10 min bez povelu, při ztrátě spojení 30 s nebo jakékoli chybě se zajistí sama.
5. **② ZAPÁLIT** (napsat `ZAPAL`) – rampa odpočítává sama (3–60 s), v T-3 s začne záznam, v T-0
   pustí proud do rozbušky na nastavený puls a zaznamenává do T-0 + *záznam po T-0* (výchozí 20 s).
6. Po testu rampa pošle výsledky (špička, impulz, doba hoření, průměrný tah, třída motoru, zpoždění zážehu,
   max. tlak a teploty) a stanice sama stáhne **ořízlý zážeh** (tah, tlak a teploty, 50 Hz) →
   `Dokumenty\GroundStation\stand\test_NNN_….csv`. Plný záznam (2000 S/s) stáhneš diagnostikou rampy přes USB
   → `Dokumenty\Rampa`.

### Zážeh přes LoRa (oříznutý, 50 Hz)

**Jak rampa pozná začátek a konec hoření** (z 100 Hz průběhu = průměry tahu po 10 ms, od T-0):

* práh = **5 % špičkového tahu**, nejméně `ign_thr_n` (výchozí 2 N);
* **začátek** = první okamžik, od kterého tah zůstane nad prahem **aspoň 30 ms** v kuse;
* **konec** = poslední okamžik úseku nad prahem dlouhého aspoň 30 ms (hledá se od konce záznamu);
* krátké špičky pod 30 ms (rána rozbušky, ťuknutí do stojanu, šum) se nepočítají; propady a „chuffing“ mezi
  začátkem a koncem patří k hoření. Doba hoření, impulz (z plné frekvence 2000 S/s) a průměrný tah se počítají
  mezi začátkem a koncem. Stejné pravidlo používá i `gui/rampa_record.py` pro plný záznam z USB.

**Co se pošle:** výřez od *začátek − 0,25 s* do *konec + 0,5 s*, 50 Hz (výřez nad 30 s 20 Hz), každý bod je
průměr tahu a tlaku za svůj interval; teploty jednou na blok. Bez zážehu (nezažehl) T-0 − 0,5 s … + 3 s,
zkušební záznam bez tahu celý. Bloky po 40 bodech (188 B, ~0,14 s na vzduchu) jdou **za sebou v jedné dávce**
(mezi bloky 40 ms poslech, každý 8. paket telemetrie); stanice pak **znovu vyžádá jen chybějící bloky**
(nejvýš 30 kol / 90 s). Zážeh 10 s ≈ 14 bloků ≈ **3 s** na vzduchu, se ztrátami pár sekund navíc.
Ručně: tlačítko *Stáhnout zážeh* / `CMD BURN [první blok] [počet]`.

**ABORT** (stanice, `Ctrl+Shift+A`, tlačítko BC na stanici, tlačítko SW1 na rampě) funguje vždy. Odpočet se
**sám přeruší**, když rampa 2,5 s neslyší stanici, při vypnutí klíče, ztrátě kontinuity, poklesu baterie,
stisku tlačítka nebo výpadku měření/záznamu. Když se rozbuška odpálí a tah nepřijde, rampa i stanice hlásí
**NEZAŽEHL** (bliká červeno-bíle) – k motoru nejméně 5 minut nechoď, pak nejdřív vypni klíč.

### Jak to funguje uvnitř

* **Jádro 1** (`thrust_stand.h`) vlastní SPI0: ADS1220 převádí tenzometr nepřetržitě 2000×/s (turbo),
  každý vzorek dostane čas, tlak (8 čtení ADC) a poslední teploty a jde do 16B rámce s CRC-8. Termistory se
  měří vložením 2 převodů každých 100 ms (při záznamu 125 ms) – mezera ~1,5 ms je vidět v časových značkách.
  Stránky po 256 B se programují mezi převody; flash se maže při ARM, takže zápis nikdy nečeká na mazání.
  **Jádro 1 se nikdy nedotkne pyro výstupů.**
* **Jádro 0** (`Rampa_MCU.ino`) dělá vše ostatní: blokování, odpočet, zážeh, LoRa, USB, signalizaci.
  LoRa vysílá asynchronně, aby T-0 nezdržel přenos paketu; 150 ms před T-0 rampa mlčí (stanice může posílat ABORT).
* **Rádio:** stejná modulace a sync word jako Zenith (433 MHz, SF7, BW 250 kHz), vlastní pakety
  (`stand_link.h`, CRC-16). Rampa poslouchá vždy, když nevysílá; stanice posílá povel hned po telemetrii
  rampy a každý povel (i PING ~2,5 Hz) je zároveň „heartbeat“.
* Signalizace: bílá = test motoru zajištěn, žlutá rychle = maže flash, zelená + pípnutí à 2 s = ODJIŠTĚNO,
  oranžová + pípání = odpočet, červená = zážeh/záznam, azurová = zkušební záznam, fialová = ABORT,
  červeno-bílé blikání = NEZAŽEHL.

### Bezpečnostní spínač otočený servem (dálkové odjištění)

Pyro napájení (J4) jde přes **dva spínače v sérii**: klíč, který sepne poslední člověk odcházející od rampy, a
spínač, kterým otočí **servo** (J12) až na povel z dálky. Rampa měří napětí až za oběma spínači („klíč“).

* Zapnutí: Rampa Console → *Serva* → vyber servo, nastav polohy SAFE/ARM (µs), *Zapsat*, pak **ARMTEST**
  (rozbuška odpojená) – rampa ukáže, za kolik ms klíč sepnul. Bez nastaveného serva vše funguje jen s klíčem.
* **Kdy servo sepne:** jen na povel k odjištění a jen když je vše ostatní v pořádku –
  test motoru: **ARM** ze stanice (LoRa rampy); raketa na pogu: **PREPARE** (Zenith READY / Ariadna „launch soon“,
  raketa ho opakuje 20×/s, rampa je READY, až když klíč opravdu sepne). Když klíč do `arm_settle_ms` nesepne
  (lidský klíč vypnutý, servo špatně nastavené), servo se vrátí a rampa hlásí chybu klíče; u rakety se znovu
  nezkouší, dokud raketa nepošle PING (SAFE / DISARM).
* **Kdy servo rozepne:** SAFE / DISARM / ABORT / PING, **hned po pulzu do rozbušky**, ztráta rakety nebo stanice,
  a vždy, když už nic nedrží rampu odjištěnou (u rakety bez povelu 1,5 s).
* ⚠ Serva na J12 jsou napájená přímo z baterie (2S, až 8,4 V) – použij **HV servo**. Ručně lze servy hýbat z
  Rampa Console (záložka *Serva*); servo bezpečnostního spínače jen když nic není odjištěné a samo se vrátí do SAFE.

### Diagnostické GUI (`gui/rampa_diag.py`, USB)

Design VoltinoLabs (jako konzole Daidalosu): přehled napájení, pyro výstupů, rakety/spojení a měřicího jádra,
kontrola „proč nejde zapálit“, živý tah/tlak/teploty, kalibrace (TARE, závaží, katalog snímače, nula a rozsah
tlakového čidla, termistory, všechny parametry), záznamy na flash (stažení v plné frekvenci, statistika, třída
motoru, CSV + BIN), **serva** (4 kanály, bezpečnostní spínač, ARMTEST), konzole. `python gui/rampa_diag.py --demo` běží bez hardwaru. Zapálit z něj nejde.

### Zkoušky před prvním zážehem

1. Nahrát firmware (Pico 2, arduino-pico ≥ 6.4) – v diagnostice: ADS1220 2000 S/s, flash `EF4018`, jádro 1 běží.
2. Bez zátěže TARE, závažím kalibrovat, ověřit druhým závažím. Tlakové čidlo na atmosféře: Nula tlaku.
3. Zkušební záznam 10 s (bez pyro), stáhnout, zkontrolovat CSV.
4. **Místo rozbušky žárovka / LED s odporem**: stanice → ARM → ZAPÁLIT 5 s; žárovka svítí právě puls.
   Vyzkoušet ABORT v odpočtu, vypnutí stanice v odpočtu (rampa sama ABORT do 2,5 s), tlačítko SW1, klíč.
5. Teprve pak motor.
