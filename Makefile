# ATtiny402 SHT40 Modbus RTU Firmware
# Toolchain: avr-gcc

MCU     = attiny402
F_CPU   = 3333333UL
TARGET  = sht40_modbus

CC      = avr-gcc
OBJCOPY = avr-objcopy
SIZE    = avr-size

CFLAGS  = -mmcu=$(MCU) -DF_CPU=$(F_CPU) -Os
CFLAGS += -Wall -Wextra -Werror -std=c11
CFLAGS += -funsigned-char -funsigned-bitfields
CFLAGS += -ffunction-sections -fdata-sections
LDFLAGS = -mmcu=$(MCU) -Wl,--gc-sections

SRC = main.c

.PHONY: all clean flash size

all: $(TARGET).hex size

$(TARGET).elf: $(SRC)
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $^

$(TARGET).hex: $(TARGET).elf
	$(OBJCOPY) -O ihex -R .eeprom $< $@

size: $(TARGET).elf
	$(SIZE) --mcu=$(MCU) -C $<

clean:
	rm -f $(TARGET).elf $(TARGET).hex

# Flash via UPDI (pymcuprog or avrdude with serialupdi)
flash: $(TARGET).hex
	pymcuprog write -t uart -u /dev/ttyUSB1 -d $(MCU) -f $< --erase --verify

# Alternative: flash via avrdude (requires avrdude 7.x with serialupdi)
flash-avrdude: $(TARGET).hex
	avrdude -c serialupdi -P /dev/ttyUSB1 -p $(MCU) -U flash:w:$<:i
