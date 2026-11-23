# USB protokol rampy (textové řádky, 115200 Bd, `\r\n`)

Klient je diagnostické GUI [`gui/rampa_diag.py`](../gui/rampa_diag.py), ale vše jde i ze Serial Monitoru
Arduino IDE. Rampa nikdy nečeká na pomalý počítač: když USB nestíhá, řádek zahodí.

> **Motor se z USB nikdy nezapálí.** `ARM` a `FIRE` rampa přes USB odmítne
> (`#ERR … only from the ground station`). Zážeh jde jen z pozemní stanice přes LoRa – z bezpečné vzdálenosti.

## Výstup

| Řádek | Kdy | Pole |
|---|---|---|
| `$ID,Rampa,<fw>,PCB1,<flash>` | na `HELLO` | verze firmwaru, `W25Q128` / `noflash` |
| `$S,…` | 5 Hz | viz níže |
| `$M,t_ms,force,fmax,psi,p_V,t1,t2,uVV` | 50 Hz (`LIVE 1`, výchozí) | tah (N, nebo µV/V bez kalibrace), max. tah za 20 ms, tlak psi, výstup čidla V, teploty °C (`nan` = nepřipojeno), signál tenzometru µV/V bez TARE |
| `$CFG,key,value` + `#OK CFG` | na `HELLO`, `CFG` a po každé změně | parametry uložené ve flash (níže) |
| `$R,…` | po každém testu, `RESULT` | test_no, flags (`SL_RF_*`), rozbuška, peak, impulse, burn_s, avg, t_peak, ign_delay, pmax_psi, t1max, t2max, curve_n, curve_hz, n_frames, sps, t0_idx, hangfire |
| `$REC,…` + `#OK LIST n` | `LIST` | test_no, adresa, bloky 64 KB, S/s, uzavřen, flags, vzorků, peak, impulse, burn_s |
| `$DBEGIN,test,addr,len` / `$D,offset,<512 hex>` / `$DEND,test,len` | `DUMP n` | celý záznam po 256 B (formát níže) |
| `# …` | události | text pro člověka (zámek rakety, ABORT, zážeh, výsledek testu) |
| `#OK …` / `#ERR …` | odpovědi | |

`$S` pole: `proto` (NONE/ZENITH/ARIADNA), stav rampy pro raketu (0–7), stav testu motoru (−1 = režim
neaktivní, jinak `sl_state_t`), flags (`SL_F_*`), faults (`SL_FLT_*`), vbat_mV, umb_mA, v5_mV, cont1..3_mV,
blokování pro raketu (`ZL_RMPF_*`), klíč, tlačítko, LoRa, režim test, ADS OK, flash OK, vzorků/s, poslední
test, volno kB, celkem kB, počet záznamů, záznamník (0 nečinný, 1 maže, 2 připraven, 3 zaznamenává),
průběh %, t_rel_ms, rozbuška, puls ms, záznam s, stáří paketu stanice ms (−1 nikdy), jádro 1 OK, JEDEC,
RSSI, ztracené vzorky, fw, puls serv 1–4 µs (0 = nedrží), servo bezpečnostního spínače (0 = žádné),
stav spínače (0 SAFE, 1 ARMING, 2 ARMED, 3 selhalo, 4 zkouška polohy), za kolik ms naposledy sepnul klíč.

## Příkazy

| Příkaz | Popis |
|---|---|
| `HELLO` | `$ID`, `$CFG`, `$S` (+ `$R`) |
| `STATUS` / `CFG` / `RESULT` | jednorázově |
| `LIVE 0\|1` | vypne / zapne `$M` |
| `TARE` | 0,5 s průměr tenzometru = nula (uloží se) |
| `CAL <N>` | známá síla na tenzometru → měřítko (po `TARE`; uloží se) |
| `LCRATED <kg> <mV/V>` | měřítko z katalogu snímače bez závaží |
| `PZERO` | čidlo tlaku na atmosféře → nula (přijme jen 0,2–1,25 V) |
| `SET <key> <value>` | změní a uloží jeden parametr |
| `DEFAULTS` | výchozí parametry, kalibrace tenzometru zůstane |
| `REC <s>` / `STOP` | zkušební záznam bez pyro (přepne rampu do režimu test motoru) |
| `LIST` / `DUMP <n>` | záznamy na flash, stažení |
| `ERASEALL YES` | smaže všechny záznamy (kalibrace zůstane) |
| `BUZZER 0\|1` | bzučák |
| `SERVO <1-4> <500-2500>` / `SERVO <1-4> OFF` | pohne servem / přestane ho držet. Servo bezpečnostního spínače jen když nic není odjištěné, mimo polohu SAFE se samo vrátí za 10 s, uvolnit ho nejde |
| `ARMTEST` | servo spínače do ARM, změří za kolik ms sepne klíč (`#OK ARMTEST …` / `#ERR ARMTEST …`), za 3 s zpět do SAFE; bez rakety a jen zajištěné |
| `STAND 1\|0` | režim test motoru zap / vyp (bez rakety) |
| `DISARM` / `ABORT` | test motoru: zpět do SAFE / přerušit |

Kalibrace a `SET` jdou jen když rampa není odjištěná, neodpočítává a nezaznamenává.

### Parametry (`$CFG`, `SET`)

| Klíč | Výchozí | Význam |
|---|---|---|
| `lc_offset` | 0 | TARE, kód ADS1220 |
| `lc_scale` | 0 | N na kód; 0 = nekalibrováno (síla v µV/V) |
| `p_zero` / `p_span` | 0,100 / 0,800 | výstup čidla / napájení při 0 psi a rozsah (0,5–4,5 V z 5 V) |
| `p_full_psi` | 500 | tlak při plném výstupu |
| `adc_vref` / `p_div` | 3,30 / 1,47 | reference ADC Pica, dělič R46/R47 |
| `th_r0` / `th_beta` / `th_t0_c` / `th_rfix` | 10000 / 3950 / 25 / 10000 | NTC termistory a R48/R49 |
| `ign_thr_n` | 2 | minimální tah, který se počítá jako hoření (práh = max(5 % špičky, tohle)) |
| `fire_ms` / `pyro_ch` / `cd_s` | 1000 / 1 / 10 | výchozí puls, rozbuška, odpočet (stanice je posílá v povelu) |
| `rec_s` | 20 | záznam po T-0 |
| `rate_2k` | 1 | 1 = 2000 S/s (ADS1220 turbo), 0 = 1000 S/s |
| `arm_servo` | 0 | servo (1–4) bezpečnostního spínače v sérii s klíčem J4; 0 = jen klíč |
| `arm_us` / `safe_us` | 2000 / 1000 | puls serva: spínač sepnut / rozepnut |
| `arm_settle_ms` | 1500 | do kdy po natočení musí klíč sepnout, jinak servo zpět a chyba klíče |

## Záznam na flash (W25Q128, `thrust_stand.h`)

```
0x000000 / 0x001000   konfigurace A / B (novější platná kopie vyhrává – výpadek napájení nevadí;
                      verze 1 bez serva se při startu převede na verzi 2, kalibrace zůstane)
0x010000 …            záznamy, každý od hranice 64 KB:
    +0    hlavička 128 B  ts_hdr_t  ("RTS1", test, bloky, S/s, druh, rozbuška, puls, napětí 5V, kopie konfigurace, CRC16)
    +128  patička  128 B  ts_end_t  ("END1", počet vzorků, vzorek T-0, výsledky, ztracené vzorky, CRC16)
                          – chybí, když vypadlo napájení; vzorky se pak čtou do první vadné CRC
    +256  vzorky po 16 B  ts_frame_t
```

`ts_frame_t` (little-endian): `uint32 t_us` od začátku záznamu, `int32 lc` kód ADS1220 (AIN1−AIN0, PGA 128),
`uint16 p` součet 8 čtení 12bit ADC ×2, `uint16 th1, th2` poměr termistoru ×65535 (poslední změřený),
`uint8 flags` (bit0 proud do rozbušky, bit1/2 nový T1/T2, bit3 bez tenzometru, bity 4–7 stav), `uint8 crc8`.

Převod (stejný ve firmwaru i v [`gui/rampa_record.py`](../gui/rampa_record.py)):

```
síla [N]   = (lc − lc_offset) · lc_scale                     (lc_scale = 0: (lc − lc_offset) · 1e6/(128·2^23) µV/V)
U_čidla    = p / (16 · 4095) · adc_vref · p_div
tlak [psi] = (U_čidla / U_5V − p_zero) / p_span · p_full_psi   (U_5V z hlavičky / patičky, ratiometricky)
R_NTC      = th_rfix · (1/poměr − 1),   T = 1 / (1/(T0+273,15) + ln(R/R0)/B) − 273,15
```

`python gui/rampa_record.py zaznam.bin` vypíše statistiku a uloží CSV.
