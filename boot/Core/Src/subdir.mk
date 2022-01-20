################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/bmp280.c \
../Core/Src/bootloader.c \
../Core/Src/dispcolor.c \
../Core/Src/f16f.c \
../Core/Src/f24f.c \
../Core/Src/f32f.c \
../Core/Src/f6x8m.c \
../Core/Src/font.c \
../Core/Src/gc9a01.c \
../Core/Src/main.c \
../Core/Src/mq135.c \
../Core/Src/si7021.c \
../Core/Src/stm32f4xx_hal_msp.c \
../Core/Src/stm32f4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f4xx.c 

OBJS += \
./Core/Src/bmp280.o \
./Core/Src/bootloader.o \
./Core/Src/dispcolor.o \
./Core/Src/f16f.o \
./Core/Src/f24f.o \
./Core/Src/f32f.o \
./Core/Src/f6x8m.o \
./Core/Src/font.o \
./Core/Src/gc9a01.o \
./Core/Src/main.o \
./Core/Src/mq135.o \
./Core/Src/si7021.o \
./Core/Src/stm32f4xx_hal_msp.o \
./Core/Src/stm32f4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f4xx.o 

C_DEPS += \
./Core/Src/bmp280.d \
./Core/Src/bootloader.d \
./Core/Src/dispcolor.d \
./Core/Src/f16f.d \
./Core/Src/f24f.d \
./Core/Src/f32f.d \
./Core/Src/f6x8m.d \
./Core/Src/font.d \
./Core/Src/gc9a01.d \
./Core/Src/main.d \
./Core/Src/mq135.d \
./Core/Src/si7021.d \
./Core/Src/stm32f4xx_hal_msp.d \
./Core/Src/stm32f4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f4xx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DBOOT_LOADER -DUSE_HAL_DRIVER -DSTM32F411xE -DDEBUG -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O2 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/bmp280.d ./Core/Src/bmp280.o ./Core/Src/bootloader.d ./Core/Src/bootloader.o ./Core/Src/dispcolor.d ./Core/Src/dispcolor.o ./Core/Src/f16f.d ./Core/Src/f16f.o ./Core/Src/f24f.d ./Core/Src/f24f.o ./Core/Src/f32f.d ./Core/Src/f32f.o ./Core/Src/f6x8m.d ./Core/Src/f6x8m.o ./Core/Src/font.d ./Core/Src/font.o ./Core/Src/gc9a01.d ./Core/Src/gc9a01.o ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/mq135.d ./Core/Src/mq135.o ./Core/Src/si7021.d ./Core/Src/si7021.o ./Core/Src/stm32f4xx_hal_msp.d ./Core/Src/stm32f4xx_hal_msp.o ./Core/Src/stm32f4xx_it.d ./Core/Src/stm32f4xx_it.o ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/system_stm32f4xx.d ./Core/Src/system_stm32f4xx.o

.PHONY: clean-Core-2f-Src

