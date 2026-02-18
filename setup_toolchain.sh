#!/bin/bash
# Install AVR toolchain (avr-gcc 15.1.0) + pymcuprog on Ubuntu/Debian.
# Usage: ./setup_toolchain.sh
set -euo pipefail

AVR_GCC_VERSION="15.2.0"
AVR_GCC_URL="https://github.com/ZakKemble/avr-gcc-build/releases/download/v${AVR_GCC_VERSION}-1/avr-gcc-${AVR_GCC_VERSION}-x64-linux.tar.bz2"
#https://github.com/ZakKemble/avr-gcc-build/releases/download/v15.2.0-1/avr-gcc-15.2.0-x64-linux.tar.bz2
INSTALL_DIR="/opt/avr-gcc-${AVR_GCC_VERSION}"

echo "=== Installing AVR toolchain (avr-gcc ${AVR_GCC_VERSION}) ==="
sudo apt-get update
sudo apt-get install -y python3-pip wget

if [ -x "${INSTALL_DIR}/bin/avr-gcc" ]; then
    echo "avr-gcc ${AVR_GCC_VERSION} already installed ✓"
else
    echo "Downloading avr-gcc ${AVR_GCC_VERSION}..."
    wget -q --show-progress "$AVR_GCC_URL" -O /tmp/avr-gcc.tar.bz2
    sudo mkdir -p "$INSTALL_DIR"
    sudo tar xjf /tmp/avr-gcc.tar.bz2 -C "$INSTALL_DIR" --strip-components=1
    rm /tmp/avr-gcc.tar.bz2
    echo "avr-gcc ${AVR_GCC_VERSION} installed to ${INSTALL_DIR} ✓"
fi

export PATH="${INSTALL_DIR}/bin:$PATH"

echo ""
echo "=== Installing pymcuprog ==="
pip install --user pymcuprog

echo ""
echo "=== Verification ==="
avr-gcc --version | head -1
echo -n "pymcuprog: "
pymcuprog --version 2>/dev/null || echo "MISSING (run: pip install pymcuprog)"

# Add to PATH permanently if not already there
PATH_LINE="export PATH=${INSTALL_DIR}/bin:\$PATH"
SHELL_RC="$HOME/.bashrc"
[ -f "$HOME/.profile" ] && [ ! -f "$HOME/.bashrc" ] && SHELL_RC="$HOME/.profile"

if grep -qF "${INSTALL_DIR}/bin" "$SHELL_RC" 2>/dev/null; then
    echo "PATH already configured in ${SHELL_RC} ✓"
else
    echo "" >> "$SHELL_RC"
    echo "# AVR toolchain (avr-gcc ${AVR_GCC_VERSION})" >> "$SHELL_RC"
    echo "$PATH_LINE" >> "$SHELL_RC"
    echo "Added to ${SHELL_RC} ✓ (restart shell or run: source ${SHELL_RC})"
fi

echo ""
echo "=== Done! ==="
echo "  Build:  make"
echo "  Flash:  make flash"
