# USB protokol rampy (textové řádky, 115200 Bd, `\n`)

GUI [`gui/rampa_diag.py`](../gui/rampa_diag.py) je klientem tohoto protokolu — vše lze
používat i přímo ze Serial Monitoru v Arduino IDE bez připojení rakety nebo pozemní stanice.

Původní 1 Hz čitelný výpis (`RAMPA scan port ...`) i výpisy událostí zůstávají zachovány,
takže dosavadní diagnostika přes Serial Monitor, Zenith i pozemní stanici funguje beze změny.
Strukturovaný proud `$S` se zapne po přijetí `HELLO` nebo `STREAM 1` a nikdy neblokuje
reálný chod rampy (pokud je USB buffer plný, řádek se zahodí).

## Výstup z rampy

| Řádek | Kdy | Pole |
|---|---|---|
| `$ID,Rampa PCB1,<fw>,Zenith+Daidalos` | na `HELLO` | identifikace desky a verze FW (`100` = `1.00`) |
| `$S,...` | 10 Hz (po `HELLO`/`STREAM 1`) nebo na `STATUS` | `t_ms, proto, port, state, flags, faults, hw_mask, vbat_mv, ibat_ma, umb_mv, umb_ma, v5_mv, c1_mv, c2_mv, c3_mv, key, btn, pyro_arm, fire_mask, lora_on, rocket_pwr, umb_trip, locate, diag, diag_arm, t_rem_ms, pyro_ch, fire_ms, last_seq, cmd_age_ms, exp_in, exp_out, lora_tx, lora_rx, conn` |
| `$I2C,0x20,0x40,...` | na `I2C SCAN` | nalezené 7bitové adresy na interní sběrnici I²C1 (GP6/GP7) |
| `$EV,t_ms,<text>` | při události (při `STREAM 1`) | události stavového automatu, LoRa a DIAG |
| `#OK ...` / `#ERR ...` | odpověď na příkaz | potvrzení nebo důvod odmítnutí |
| `RAMPA ...` | 1 Hz vždy | původní lidsky čitelný výpis (zachován) |

Bitové masky:
- `proto`: `0` scan, `1` Zenith (UART), `2` Daidalos/Ariadna (I²C)
- `conn`: `0` scan (`J3` + `J14`), `1` konektor `J3` (`GP0/GP1`), `2` konektor `J14` (`GP8/GP9`)
- `port`: `0` Hi-Z, `1` `J3` UART0, `2` `J3` I²C0, `3` `J14` UART1, `4` `J14` I²C0, `5` Scan A (`J3` UART0 + `J14` I²C0), `6` Scan B (`J3` I²C0 + `J14` UART1)
- `state`, `flags`, `faults`: shodné s `zenith_link.h` (`ZR_ST_*`, `ZL_RMP_*`, `ZL_RMPF_*`)
- `hw_mask`: bit0 `TCA9534 (0x20)`, bit1 `INA226 VBAT (0x40)`, bit2 `INA226 UMB (0x41)`, bit3 `ADS1115 (0x48)`, bit4 `LoRa SX1278`
- `fire_mask`: bit0–2 aktuálně sepnutý kanál `CH1–CH3` (GP16–GP18)

## Příkazy

| Příkaz | Kdy | Popis |
|---|---|---|
| `HELLO` | vždy | vypíše `$ID`, zapne 10 Hz `$S` a pošle okamžitý stav |
| `STATUS` / `PING` | vždy | pošle jeden řádek `$S` |
| `STREAM 0\|1` | vždy | vypne / zapne 10 Hz proud `$S` |
| `I2C SCAN` | mimo odpočet/zážeh | oskenuje interní sběrnici I²C1 (`0x08–0x77`) |
| `ROCKETPWR 0\|1` | vždy | vypne / zapne požadavek na `UMB_5V` (Ariadnu nenapájí) a resetuje nadproud |
| `BUZZ [ms]` | vždy | krátce pípne bzučákem (`50–5000 ms`, výchozí `300 ms`) |
| `LOCATE 0\|1` | vždy | zapne / vypne vyhledávací pípání |
| `DIAG 1\|0` | bez připojené rakety | zapne / vypne stolní diagnostický režim |
| `ARM 1\|0` | jen v `DIAG` (`ARM 0` vždy) | zapne / vypne pyro lištu `GP19` (vyžaduje klíč J4, bez tlačítka ABORT, `VBAT ≥ 6,6 V`) |
| `FIRE <1..3> [ms] CONFIRM` | jen v `DIAG` + `ARM 1` | zkušební pulz na kanálu `1..3` (`50–2000 ms`, výchozí `300 ms`) |
| `LED 0\|1\|AUTO` | jen v `DIAG` | ruční test červeného světla (TCA9534 P6) |
| `RGB AUTO\|OFF\|R\|G\|B\|Y\|C\|M\|W` | jen v `DIAG` | ruční test SK6812 RGB LED (GP20) |
| `LORA AUTO\|ON\|OFF\|BEACON` | jen v `DIAG` | test napájení/inicializace Ra-02 nebo okamžitý testovací beacon |
| `ABORT` | vždy | okamžitě ukončí `DIAG`, vypne pyro a přejde do `ABORT` |
| `SAFE` | vždy | ukončí `DIAG`, vypne pyro, smaže `ABORT` a přejde do `IDLE` |
| `HELP` | vždy | vypíše stručný seznam příkazů |

## Bezpečnostní pojistky v `DIAG`

- Stisk lokálního tlačítka `ABORT` (`SW1`) okamžitě vypne pyro, ukončí `DIAG` a přejde do `ABORT`.
- Rozepnutí klíče `J4` nebo pokles `VBAT < 6,6 V` okamžitě shodí `ARM` a vypne pyro.
- Připojení rakety (Zenith nebo Daidalos) na pogo okamžitě ukončí `DIAG` a předá řízení raketě.
