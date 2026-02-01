# -------- Project configuration --------
PROJECT = main
MCU     = atmega328p

CC      = avr-gcc
OBJCOPY = avr-objcopy
OBJDUMP = avr-objdump
AVRDUDE = sudo avrdude

# -------- Flags --------
CFLAGS  = -Os -mmcu=$(MCU) -Wall -Wextra
LDFLAGS = -mmcu=$(MCU)

# -------- Files --------
SRC     := $(wildcard *.c)
OBJ     := $(SRC:.c=.o)

ELF     = $(PROJECT).elf
HEX     = $(PROJECT).hex
DUMP    = $(PROJECT).dump

# -------- Targets --------
.PHONY: all build flash clean

all: build

build: $(ELF) $(HEX) $(DUMP)

# Compile each .c into .o
%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

# Link
$(ELF): $(OBJ)
	$(CC) $(LDFLAGS) $^ -o $@

# Create HEX
$(HEX): $(ELF)
	$(OBJCOPY) -O ihex -R .eeprom $< $@

# Create disassembly
$(DUMP): $(ELF)
	$(OBJDUMP) -h -S $< > $@

# Flash to MCU
flash: $(HEX)
	$(AVRDUDE) -p m328p -c usbasp -U flash:w:$<

# -------- Fuse programming --------

# Internal RC 8MHz, CKDIV8 disabled (recommended)
fuses-8mhz:
	$(AVRDUDE) -p $(PART) -c $(PROGRAMMER) \
		-U lfuse:w:0xE2:m \
		-U hfuse:w:0xD9:m \
		-U efuse:w:0xFF:m

# Internal RC 8MHz, CKDIV8 enabled (1MHz system clock)
fuses-1mhz:
	$(AVRDUDE) -p $(PART) -c $(PROGRAMMER) \
		-U lfuse:w:0x62:m \
		-U hfuse:w:0xD9:m \
		-U efuse:w:0xFF:m

# Clean build artifacts
clean:
	rm -f $(OBJ) $(ELF) $(HEX) $(DUMP)
