#****************************************************************************
# Makefile
# openacousticdevices.info
# November 2020
#****************************************************************************

# Set the toolpath parameter to the location of the GNU Arm Embedded Toolchain

TOOLPATH = 

# These the locations of the source and header files

INC = ../cmsis ../device/inc ../emlib/inc ../emusb/inc ../drivers/inc ../fatfs/inc  ../inc
SRC = ../device/src ../emlib/src ../emusb/src ../drivers/src ../fatfs/src  ../src

# Set the name of the output files

FILENAME = audiomoth

# This is the location of the resulting object files

OBJPATH = ./objects/

# Set the target EFM32WG to that used in AudioMoth

TARGET = EFM32WG380F256

# The following code generates the list of objects and the search path of source and header files

VPATH = $(SRC)

IFLAGS = $(foreach d, $(INC), -I$d)

_CSRC = $(notdir $(foreach d, $(SRC), $(wildcard $d/*.c)))
_SSRC = $(notdir $(foreach d, $(SRC), $(wildcard $d/*.s)))

_OBJ = $(_CSRC:.c=.o) $(_SSRC:.s=.o)

OBJ = $(sort $(foreach d, $(_OBJ), $(OBJPATH)$d))

DEP = $(OBJ:.o=.d)

# These are the compilation settings

CC = $(TOOLPATH)arm-none-eabi-gcc

CSIZE = $(TOOLPATH)arm-none-eabi-size

COBJCOPY = $(TOOLPATH)arm-none-eabi-objcopy

CFLAGS = -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16 -mthumb -Wall '-DARM_MATH_CM4=1' '-D$(TARGET)=1'

DFLAGS = -MMD

# Finally the build rules

$(OBJPATH)%.o: %.c
	@mkdir -p $(OBJPATH)
	@echo 'Building' $@
	@$(CC) $(CFLAGS) $(DFLAGS) -O3 -ffunction-sections -fdata-sections -std=c99 -c -o "$@" "$<" $(IFLAGS)

$(OBJPATH)%.o: %.s
	@mkdir -p $(OBJPATH)
	@echo 'Building' $@
	@$(CC) $(CFLAGS) $(DFLAGS) -x assembler-with-cpp -c -o "$@" "$<" $(IFLAGS)

$(FILENAME).bin: $(OBJ)
	@echo 'Building' $(FILENAME).axf
	@$(CC) $(CFLAGS) $(DFLAGS) -T "audiomoth.ld" -Xlinker --gc-sections -Xlinker -Map="$(FILENAME).map" --specs=nano.specs -o $(FILENAME).axf $(OBJ) -lm -Wl,--start-group -lgcc -lc -lnosys -Wl,--end-group
	@echo 'Building' $(FILENAME).hex
	@$(COBJCOPY) -O ihex "$(FILENAME).axf" "$(FILENAME).hex"
	@echo 'Building' $(FILENAME).bin
	@$(COBJCOPY) -O binary "$(FILENAME).axf" "$(FILENAME).bin"
	@$(CSIZE) -A "$(FILENAME).axf"

-include $(DEP)

.PHONY: clean
clean:
	rm -f $(OBJPATH)*.o
	rm -f $(OBJPATH)*.d
	rm -f $(FILENAME).axf
	rm -f $(FILENAME).bin
	rm -f $(FILENAME).hex
	rm -f $(FILENAME).map
