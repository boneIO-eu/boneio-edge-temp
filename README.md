# boneIO edge-temp

Czujnik temperatury i wilgotności SHT40 na ATtiny402 z komunikacją Modbus RTU po RS-485.

## Mapa rejestrów

| Adres | Nazwa       | Jednostka | R/W | Opis                                |
| ----- | ----------- | --------- | --- | ----------------------------------- |
| 0     | Humidity    | 0.1 %RH   | R   | Wilgotność (S_WORD, `× 0.1` = %)    |
| 1     | Temperature | 0.1 °C    | R   | Temperatura (S_WORD, `× 0.1` = °C)  |
| 100   | Slave ID    | 1–247     | R/W | Adres Modbus                        |
| 101   | Baud Rate   | 0–3       | R/W | 0=2400, 1=4800, **2=9600**, 3=19200 |
| 102   | LED mode    | 0–2       | R/W | 0=off, 1=on, **2=auto**             |
| 103   | FW version  | —         | R   | Np. `0x0001` = v0.1                 |
| 104   | Temp cal    | 0.1 °C    | R/W | Offset kalibracji temp (±50.0)      |
| 105   | Hum cal     | 0.1 %RH   | R/W | Offset kalibracji wilg. (±50.0)     |

Domyślnie: adres **1**, baudrate **9600**, LED **auto**.

## Kompilacja

### Wymagania

- `avr-gcc` + `avr-libc`
- `pymcuprog` lub `avrdude` ≥ 7.x (do flashowania przez UPDI)

### Linux (Ubuntu/Debian)

```bash
# Instalacja toolchaina
sudo apt install gcc-avr avr-libc

# Instalacja programatora UPDI
pip install pymcuprog

# Kompilacja
make

# Sprawdzenie rozmiaru (musi się zmieścić w 4KB flash / 256B RAM)
make size
```

### Flashowanie przez UPDI

Potrzebujesz programatora UPDI (np. UPDI Friend, SerialUPDI) podłączonego do pinu UPDI (PA0) na złączu H1.

> ⚠️ **Usuń rezystor R20 (470Ω)** z linii UPDI i zastąp zworką — UPDI Friend ma już rezystor na pokładzie.

```bash
# Przez pymcuprog (zalecane)
make flash

# Lub przez avrdude 7.x
make flash-avrdude
```

Domyślny port: `/dev/ttyUSB0`. Aby zmienić:

```bash
pymcuprog write -t uart -u /dev/ttyACM0 -d attiny402 -f sht40_modbus.hex
```

### VSCode

1. Zainstaluj rozszerzenie **PlatformIO** lub **AVR Helper**.

2. Otwórz ten folder w VSCode.

3. **Terminal → New Terminal**, wpisz:

   ```bash
   make          # kompilacja
   make flash    # wgranie
   ```

4. Alternatywnie — można użyć **Tasks** (`.vscode/tasks.json`):
   ```json
   {
     "version": "2.0.0",
     "tasks": [
       {
         "label": "Build",
         "type": "shell",
         "command": "make",
         "group": { "kind": "build", "isDefault": true }
       },
       {
         "label": "Flash",
         "type": "shell",
         "command": "make flash"
       }
     ]
   }
   ```
   Potem: `Ctrl+Shift+B` → Build, albo `Ctrl+Shift+P` → "Run Task" → Flash.

## CI/CD

GitHub Action automatycznie buduje firmware przy każdym pushu.

Aby stworzyć nowy **Release z binarką**:

```bash
git tag v0.1
git push --tags
```

Binarka `.hex` pojawi się w zakładce **Releases** na GitHubie.

## Hardware

- **MCU:** ATtiny402-SSFR (3.3V, 8-pin)
- **Sensor:** SHT40-AD1B-R3 (I2C, adres 0x44)
- **RS-485:** THVD1406DR (auto-direction, bez DE/RE)
- **LED:** NCD0402G1 (active low na PA3)

### Pinout ATtiny402

| Pin | Funkcja             |
| --- | ------------------- |
| PA0 | UPDI                |
| PA1 | SDA (I2C → SHT40)   |
| PA2 | SCL (I2C → SHT40)   |
| PA3 | LED                 |
| PA6 | TXD (UART → RS-485) |
| PA7 | RXD (UART ← RS-485) |

## Licencja

GPL-3.0 — patrz [LICENSE](LICENSE).
