#!/bin/bash
# Flash boneIO Edge Temp boards one after another.
# USB programmer stays connected — just swap the target board.
#
# Usage: ./flash_auto.sh [hex_file] [port]
#
# Workflow:
#   1. Connect UPDI programmer to PC (stays connected)
#   2. Run this script
#   3. Connect target board → script detects + flashes
#   4. Disconnect board, connect next → repeat

set -euo pipefail

HEX="${1:-sht40_modbus.hex}"
PORT="${2:-/dev/ttyUSB1}"
MCU="attiny402"

if [ ! -f "$HEX" ]; then
    echo "❌ Hex file not found: $HEX"
    echo "   Run 'make' first."
    exit 1
fi

COUNTER=0
POLL_INTERVAL=1  # seconds between ping attempts

beep_ok()   { echo -e "\a"; }
beep_fail() { echo -e "\a\a\a"; }

flash_device() {
    COUNTER=$((COUNTER + 1))

    echo ""
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo "  📟 Board #${COUNTER} — flashing..."
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

    if pymcuprog write -t uart -u "$PORT" -d "$MCU" -f "$HEX" --erase --verify 2>&1; then
        echo -e "  \033[32m✅ #${COUNTER} OK!\033[0m"
        beep_ok
    else
        echo -e "  \033[31m❌ #${COUNTER} FAILED!\033[0m"
        beep_fail
    fi

    echo ""
    echo "  🔌 Swap board and wait..."

    # Wait until board is DISCONNECTED (ping fails)
    while pymcuprog ping -t uart -u "$PORT" -d "$MCU" >/dev/null 2>&1; do
        sleep 0.5
    done
    echo "  ⏳ Board removed, waiting for next..."
}

echo "╔══════════════════════════════════════════════════╗"
echo "║  boneIO Edge Temp — Production Flasher          ║"
echo "║  Firmware: ${HEX}"
echo "║  Port:     ${PORT}"
echo "║  Press Ctrl+C to stop                           ║"
echo "╚══════════════════════════════════════════════════╝"
echo ""
echo "  ⏳ Waiting for first board..."

while true; do
    # Wait until a board is CONNECTED (ping succeeds)
    if pymcuprog ping -t uart -u "$PORT" -d "$MCU" >/dev/null 2>&1; then
        flash_device
    fi
    sleep "$POLL_INTERVAL"
done
