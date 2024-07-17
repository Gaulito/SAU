################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/NMEA.c \
../Core/Src/UartRingbuffer.c \
../Core/Src/bmp180.c \
../Core/Src/hmc5883.c \
../Core/Src/kalman.c \
../Core/Src/main.c \
../Core/Src/mpu6050.c \
../Core/Src/ms4525.c \
../Core/Src/stm32f3xx_hal_msp.c \
../Core/Src/stm32f3xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f3xx.c \
../Core/Src/trajectory.c 

OBJS += \
./Core/Src/NMEA.o \
./Core/Src/UartRingbuffer.o \
./Core/Src/bmp180.o \
./Core/Src/hmc5883.o \
./Core/Src/kalman.o \
./Core/Src/main.o \
./Core/Src/mpu6050.o \
./Core/Src/ms4525.o \
./Core/Src/stm32f3xx_hal_msp.o \
./Core/Src/stm32f3xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f3xx.o \
./Core/Src/trajectory.o 

C_DEPS += \
./Core/Src/NMEA.d \
./Core/Src/UartRingbuffer.d \
./Core/Src/bmp180.d \
./Core/Src/hmc5883.d \
./Core/Src/kalman.d \
./Core/Src/main.d \
./Core/Src/mpu6050.d \
./Core/Src/ms4525.d \
./Core/Src/stm32f3xx_hal_msp.d \
./Core/Src/stm32f3xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f3xx.d \
./Core/Src/trajectory.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F303xC -c -I../Core/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../Drivers/CMSIS/Include -Og -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/NMEA.cyclo ./Core/Src/NMEA.d ./Core/Src/NMEA.o ./Core/Src/NMEA.su ./Core/Src/UartRingbuffer.cyclo ./Core/Src/UartRingbuffer.d ./Core/Src/UartRingbuffer.o ./Core/Src/UartRingbuffer.su ./Core/Src/bmp180.cyclo ./Core/Src/bmp180.d ./Core/Src/bmp180.o ./Core/Src/bmp180.su ./Core/Src/hmc5883.cyclo ./Core/Src/hmc5883.d ./Core/Src/hmc5883.o ./Core/Src/hmc5883.su ./Core/Src/kalman.cyclo ./Core/Src/kalman.d ./Core/Src/kalman.o ./Core/Src/kalman.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/mpu6050.cyclo ./Core/Src/mpu6050.d ./Core/Src/mpu6050.o ./Core/Src/mpu6050.su ./Core/Src/ms4525.cyclo ./Core/Src/ms4525.d ./Core/Src/ms4525.o ./Core/Src/ms4525.su ./Core/Src/stm32f3xx_hal_msp.cyclo ./Core/Src/stm32f3xx_hal_msp.d ./Core/Src/stm32f3xx_hal_msp.o ./Core/Src/stm32f3xx_hal_msp.su ./Core/Src/stm32f3xx_it.cyclo ./Core/Src/stm32f3xx_it.d ./Core/Src/stm32f3xx_it.o ./Core/Src/stm32f3xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f3xx.cyclo ./Core/Src/system_stm32f3xx.d ./Core/Src/system_stm32f3xx.o ./Core/Src/system_stm32f3xx.su ./Core/Src/trajectory.cyclo ./Core/Src/trajectory.d ./Core/Src/trajectory.o ./Core/Src/trajectory.su

.PHONY: clean-Core-2f-Src

