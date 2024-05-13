CC=avr-gcc
OBJCOPY=avr-objcopy
FLASHER=avrdude
ENV=WIN

CFLAGS=-Os -DF_CPU=16000000UL -mmcu=atmega328p
INCLUDES=src/*.c
BUILD_PATH=build
BUILD_NAME=main

LINUX_PORT=/dev/ttyUSB1
WINDOWS_PORT=COM20
PARTNO=atmega328p
BAUD=57600
PROGRAMMER=arduino

all: build

build: compile
	${OBJCOPY} ${BUILD_PATH}/${BUILD_NAME}.elf -O ihex ${BUILD_PATH}/${BUILD_NAME}.hex
	
compile: 
	@mkdir -p build
	${CC} ${BUILD_NAME}.c ${INCLUDES} -o ${BUILD_PATH}/${BUILD_NAME}.elf ${CFLAGS} 

ison:
ifeq (${ENV}, LINUX)
	@echo "LINUX"
	${FLASHER} -c ${PROGRAMMER} -p ${PARTNO} -P ${LINUX_PORT} -b ${BAUD}
endif
ifeq (${ENV}, WIN)
	${FLASHER}.exe -c ${PROGRAMMER} -p ${PARTNO} -P ${WINDOWS_PORT} -b ${BAUD}
endif

flash: build 
ifeq (${ENV}, LINUX)
	@echo "LINUX"
	${FLASHER} -c ${PROGRAMMER} -p ${PARTNO} -P ${LINUX_PORT} -b ${BAUD} -U flash:w:${BUILD_PATH}/${BUILD_NAME}.hex:a
endif
ifeq (${ENV}, WIN)
	${FLASHER}.exe -c ${PROGRAMMER} -p ${PARTNO} -P ${WINDOWS_PORT} -b ${BAUD} -U flash:w:${BUILD_PATH}/${BUILD_NAME}.hex:a
endif
	@make clean


clean:
	@rm ${BUILD_PATH}/*.hex ${BUILD_PATH}/*.elf
