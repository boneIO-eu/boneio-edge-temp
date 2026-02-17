#!/bin/bash
# Install AVR toolchain + ATtiny402 support on Ubuntu/Debian.
# Usage: ./setup_toolchain.sh
set -euo pipefail

echo "=== Installing AVR toolchain ==="
sudo apt-get update
sudo apt-get install -y gcc-avr avr-libc binutils-avr python3-pip unzip wget
pip install --user pymcuprog

echo ""
echo "=== Checking ATtiny402 support ==="
GCC_DIR=$(find /usr/lib/gcc/avr -maxdepth 1 -type d | tail -1)

if [ -f "$GCC_DIR/device-specs/specs-attiny402" ]; then
    echo "ATtiny402 device specs already installed ✓"
else
    echo "ATtiny402 device specs not found — installing Microchip ATtiny DFP..."

    DFP_URL="https://packs.download.microchip.com/Microchip.ATtiny_DFP.3.1.260.atpack"
    TMP="/tmp/attiny-dfp"

    wget -q --show-progress "$DFP_URL" -O /tmp/attiny-dfp.zip
    mkdir -p "$TMP"
    unzip -qo /tmp/attiny-dfp.zip -d "$TMP"

    sudo cp "$TMP/gcc/dev/attiny402/device-specs/specs-attiny402" "$GCC_DIR/device-specs/"
    sudo cp "$TMP/include/avr/iotn402.h" /usr/lib/avr/include/avr/
    sudo mkdir -p /usr/lib/avr/lib/avrxmega3/short-calls
    sudo cp "$TMP"/gcc/dev/attiny402/avrxmega3/*.{a,o} /usr/lib/avr/lib/avrxmega3/ 2>/dev/null || true
    sudo cp "$TMP"/gcc/dev/attiny402/avrxmega3/short-calls/*.{a,o} /usr/lib/avr/lib/avrxmega3/short-calls/ 2>/dev/null || true

    rm -rf /tmp/attiny-dfp.zip "$TMP"
    echo "ATtiny DFP installed ✓"
fi

echo ""
echo "=== Verification ==="
avr-gcc --version | head -1
echo -n "ATtiny402 specs: "
[ -f "$GCC_DIR/device-specs/specs-attiny402" ] && echo "OK ✓" || echo "MISSING ✗"
echo -n "pymcuprog: "
pymcuprog --version 2>/dev/null || echo "MISSING (run: pip install pymcuprog)"

echo ""
echo "=== Done! ==="
echo "  Build:  make"
echo "  Flash:  make flash"
