# boneIO Edge Temp

SHT40 temperature & humidity sensor on ATtiny402 with Modbus RTU over RS-485.

## Register Map

| Addr | Name        | Unit    | R/W | Description                          |
| ---- | ----------- | ------- | --- | ------------------------------------ |
| 0    | Humidity    | 0.1 %RH | R   | Humidity (S_WORD, `× 0.1` = %)       |
| 1    | Temperature | 0.1 °C  | R   | Temperature (S_WORD, `× 0.1` = °C)   |
| 100  | Slave ID    | 1–247   | R/W | Modbus address                       |
| 101  | Baud Rate   | 0–3     | R/W | 0=2400, 1=4800, **2=9600**, 3=19200  |
| 102  | LED mode    | 0–2     | R/W | 0=off, 1=on, **2=auto**              |
| 103  | FW version  | —       | R   | e.g. `0x0001` = v0.1                 |
| 104  | Temp cal    | 0.1 °C  | R/W | Temperature calibration offset (±50) |
| 105  | Hum cal     | 0.1 %RH | R/W | Humidity calibration offset (±50)    |

Defaults: address **1**, baud **9600**, LED **auto**.

## Build

### Requirements

- `avr-gcc` + `avr-libc`
- `pymcuprog` or `avrdude` ≥ 7.x (for UPDI flashing)

### Linux (Ubuntu/Debian)

```bash
sudo apt install gcc-avr avr-libc
pip install pymcuprog

make            # compile
make size       # check size (max 4KB flash / 256B RAM)
make flash      # erase + write + verify via UPDI
```

### Flashing via UPDI

Connect a UPDI programmer (e.g. UPDI Friend, SerialUPDI) to UPDI pin (PA0) on header H1.

> ⚠️ **Remove R20 (470Ω)** from the UPDI line and replace with a solder bridge — the UPDI Friend already has its own resistor.

**Linux** (default port `/dev/ttyUSB1`):

```bash
pymcuprog write -t uart -u /dev/ttyUSB1 -d attiny402 -f sht40_modbus.hex --erase --verify
```

**Windows** — no compiler needed, just flash the `.hex` from GitHub Releases:

```powershell
# Install pymcuprog (requires Python 3)
pip install pymcuprog

# Flash (replace COM3 with your port from Device Manager)
pymcuprog write -t uart -u COM3 -d attiny402 -f boneio-edge-temp-v0.1.hex --erase --verify
```

> 💡 Find your COM port in **Device Manager → Ports (COM & LPT)** after plugging in the UPDI programmer.

### VSCode

1. Install **PlatformIO** or **AVR Helper** extension.
2. Open this folder in VSCode.
3. **Terminal → New Terminal**:
   ```bash
   make          # build
   make flash    # flash
   ```

## CI/CD

GitHub Action builds firmware automatically on every push.

To create a **Release with binary**:

```bash
git tag v0.1
git push --tags
```

The `.hex` binary will appear under **Releases** on GitHub.

## Hardware

- **MCU:** ATtiny402-SSFR (3.3V, 8-pin)
- **Sensor:** SHT40-AD1B-R3 (I2C, address 0x44)
- **RS-485:** THVD1406DR (auto-direction, no DE/RE)
- **LED:** NCD0402G1 (active low on PA3)

### ATtiny402 Pinout

| Pin | Function            |
| --- | ------------------- |
| PA0 | UPDI                |
| PA1 | SDA (I2C → SHT40)   |
| PA2 | SCL (I2C → SHT40)   |
| PA3 | LED                 |
| PA6 | TXD (UART → RS-485) |
| PA7 | RXD (UART ← RS-485) |

## License

GPL-3.0 — Copyright (C) 2025 boneIO Sp. z o.o. Made in Poland.

See [LICENSE](LICENSE).

---

# 🇵🇱 Dokumentacja po polsku

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

## Kompilacja i flashowanie

```bash
# Instalacja toolchaina
sudo apt install gcc-avr avr-libc
pip install pymcuprog

# Kompilacja
make

# Sprawdzenie rozmiaru (max 4KB flash / 256B RAM)
make size

# Flashowanie przez UPDI (erase + write + verify)
make flash
```

> ⚠️ **Usuń rezystor R20 (470Ω)** z linii UPDI i zastąp zworką.

## CI/CD

GitHub Action buduje firmware automatycznie. Aby stworzyć **Release z binarką**:

```bash
git tag v0.1
git push --tags
```

## Licencja

GPL-3.0 — Copyright (C) 2025 boneIO Sp. z o.o. Made in Poland.
