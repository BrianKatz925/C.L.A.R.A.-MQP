################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL := cmd.exe
RM := rm -rf

USER_OBJS :=

LIBS := 
PROJ := 

O_SRCS := 
C_SRCS := 
S_SRCS := 
S_UPPER_SRCS := 
OBJ_SRCS := 
ASM_SRCS := 
PREPROCESSING_SRCS := 
OBJS := 
OBJS_AS_ARGS := 
C_DEPS := 
C_DEPS_AS_ARGS := 
EXECUTABLES := 
OUTPUT_FILE_PATH :=
OUTPUT_FILE_PATH_AS_ARGS :=
AVR_APP_PATH :=$$$AVR_APP_PATH$$$
QUOTE := "
ADDITIONAL_DEPENDENCIES:=
OUTPUT_FILE_DEP:=
LIB_DEP:=
LINKER_SCRIPT_DEP:=

# Every subdirectory with source files must be described here
SUBDIRS := 


# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS +=  \
../Sketch.cpp


PREPROCESSING_SRCS += 


ASM_SRCS += 


OBJS +=  \
Sketch.o

OBJS_AS_ARGS +=  \
Sketch.o

C_DEPS +=  \
Sketch.d

C_DEPS_AS_ARGS +=  \
Sketch.d

OUTPUT_FILE_PATH +=triple_motor_control_with_ls7366r_and_cur_sensing_atmega32u4.elf

OUTPUT_FILE_PATH_AS_ARGS +=triple_motor_control_with_ls7366r_and_cur_sensing_atmega32u4.elf

ADDITIONAL_DEPENDENCIES:=

OUTPUT_FILE_DEP:= ./makedep.mk

LIB_DEP+=  \
C:/Users/sun/Desktop/documents/SRL/Salamander/code/firmware/firmware_recent/test/motor_test/ArduinoSketch2/ArduinoCore/Debug/libArduinoCore.a

LINKER_SCRIPT_DEP+= 


# AVR32/GNU C Compiler
./Sketch.o: .././Sketch.cpp
	@echo Building file: $<
	@echo Invoking: AVR8/GNU C Compiler : 5.4.0
	$(QUOTE)D:\Program Files (x86)\Atmel\Studio\7.0\toolchain\avr8\avr8-gnu-toolchain\bin\avr-g++.exe$(QUOTE) -funsigned-char -funsigned-bitfields -DDEBUG -DF_CPU=16000000L -DARDUINO=106011 -DARDUINO_AVR_MICRO -DARDUINO_ARCH_AVR -DUSB_VID=0x2341 -DUSB_PID=0x8037 -DUSB_PRODUCT="\"Arduino Micro\"" -DUSB_MANUFACTURER="\"Arduino LLC\""  -I"..\\..\ArduinoCore\include\core" -I"..\\..\ArduinoCore\include\libraries\pid_v1" -I"..\\..\ArduinoCore\include\libraries\SPI" -I"..\\..\ArduinoCore\include\libraries\Wire" -I"..\\..\ArduinoCore\include\libraries\Wire\utility" -I"..\\..\ArduinoCore\include\variants\leonardo" -I"..\\..\ArduinoCore\include\variants\micro" -I"D:\Program Files (x86)\Atmel\Studio\7.0\Packs\Atmel\ATmega_DFP\1.2.132\include"  -Os -fno-threadsafe-statics -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -g2 -Wall -w -mmcu=atmega32u4 -B "D:\Program Files (x86)\Atmel\Studio\7.0\Packs\Atmel\ATmega_DFP\1.2.132\gcc\dev\atmega32u4" -c -std=gnu++11 -MD -MP -MF "$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)"   -o "$@" "$<" 
	@echo Finished building: $<
	




# AVR32/GNU Preprocessing Assembler



# AVR32/GNU Assembler




ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: $(OUTPUT_FILE_PATH) $(ADDITIONAL_DEPENDENCIES)

$(OUTPUT_FILE_PATH): $(OBJS) $(USER_OBJS) $(OUTPUT_FILE_DEP) $(LIB_DEP) $(LINKER_SCRIPT_DEP)
	@echo Building target: $@
	@echo Invoking: AVR8/GNU Linker : 5.4.0
	$(QUOTE)D:\Program Files (x86)\Atmel\Studio\7.0\toolchain\avr8\avr8-gnu-toolchain\bin\avr-g++.exe$(QUOTE) -o$(OUTPUT_FILE_PATH_AS_ARGS) $(OBJS_AS_ARGS) $(USER_OBJS) $(LIBS) -Wl,-Map="triple_motor_control_with_ls7366r_and_cur_sensing_atmega32u4.map" -Wl,--start-group -Wl,-lm -Wl,-lArduinoCore  -Wl,--end-group -Wl,-L"C:\Users\sun\Desktop\DOCUME~1\SRL\SALAMA~1\code\firmware\FIRMWA~1\test\MOTOR_~1\ARDUIN~1\ARDUIN~1\Debug"  -Wl,--gc-sections -mmcu=atmega32u4 -B "D:\Program Files (x86)\Atmel\Studio\7.0\Packs\Atmel\ATmega_DFP\1.2.132\gcc\dev\atmega32u4" -Os  
	@echo Finished building target: $@
	"D:\Program Files (x86)\Atmel\Studio\7.0\toolchain\avr8\avr8-gnu-toolchain\bin\avr-objcopy.exe" -O ihex -R .eeprom -R .fuse -R .lock -R .signature -R .user_signatures  "triple_motor_control_with_ls7366r_and_cur_sensing_atmega32u4.elf" "triple_motor_control_with_ls7366r_and_cur_sensing_atmega32u4.hex"
	"D:\Program Files (x86)\Atmel\Studio\7.0\toolchain\avr8\avr8-gnu-toolchain\bin\avr-objcopy.exe" -j .eeprom  --set-section-flags=.eeprom=alloc,load --change-section-lma .eeprom=0  --no-change-warnings -O ihex "triple_motor_control_with_ls7366r_and_cur_sensing_atmega32u4.elf" "triple_motor_control_with_ls7366r_and_cur_sensing_atmega32u4.eep" || exit 0
	"D:\Program Files (x86)\Atmel\Studio\7.0\toolchain\avr8\avr8-gnu-toolchain\bin\avr-objdump.exe" -h -S "triple_motor_control_with_ls7366r_and_cur_sensing_atmega32u4.elf" > "triple_motor_control_with_ls7366r_and_cur_sensing_atmega32u4.lss"
	"D:\Program Files (x86)\Atmel\Studio\7.0\toolchain\avr8\avr8-gnu-toolchain\bin\avr-objcopy.exe" -O srec -R .eeprom -R .fuse -R .lock -R .signature -R .user_signatures "triple_motor_control_with_ls7366r_and_cur_sensing_atmega32u4.elf" "triple_motor_control_with_ls7366r_and_cur_sensing_atmega32u4.srec"
	"D:\Program Files (x86)\Atmel\Studio\7.0\toolchain\avr8\avr8-gnu-toolchain\bin\avr-size.exe" "triple_motor_control_with_ls7366r_and_cur_sensing_atmega32u4.elf"
	
	





# Other Targets
clean:
	-$(RM) $(OBJS_AS_ARGS) $(EXECUTABLES)  
	-$(RM) $(C_DEPS_AS_ARGS)   
	rm -rf "triple_motor_control_with_ls7366r_and_cur_sensing_atmega32u4.elf" "triple_motor_control_with_ls7366r_and_cur_sensing_atmega32u4.a" "triple_motor_control_with_ls7366r_and_cur_sensing_atmega32u4.hex" "triple_motor_control_with_ls7366r_and_cur_sensing_atmega32u4.lss" "triple_motor_control_with_ls7366r_and_cur_sensing_atmega32u4.eep" "triple_motor_control_with_ls7366r_and_cur_sensing_atmega32u4.map" "triple_motor_control_with_ls7366r_and_cur_sensing_atmega32u4.srec" "triple_motor_control_with_ls7366r_and_cur_sensing_atmega32u4.usersignatures"
	