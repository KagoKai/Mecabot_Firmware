################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Core/Src/IIR_filter.cpp \
C:/Users/ADMIN/Desktop/DATN/Mecabot_Firmware/components/rosserial/duration.cpp \
../Core/Src/mainpp.cpp \
../Core/Src/mecabot_hardware.cpp \
C:/Users/ADMIN/Desktop/DATN/Mecabot_Firmware/components/rosserial/time.cpp 

C_SRCS += \
../Core/Src/dc_motor.c \
../Core/Src/encoder.c \
../Core/Src/main.c \
../Core/Src/pid.c \
../Core/Src/stm32f1xx_hal_msp.c \
../Core/Src/stm32f1xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f1xx.c 

C_DEPS += \
./Core/Src/dc_motor.d \
./Core/Src/encoder.d \
./Core/Src/main.d \
./Core/Src/pid.d \
./Core/Src/stm32f1xx_hal_msp.d \
./Core/Src/stm32f1xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f1xx.d 

OBJS += \
./Core/Src/IIR_filter.o \
./Core/Src/dc_motor.o \
./Core/Src/duration.o \
./Core/Src/encoder.o \
./Core/Src/main.o \
./Core/Src/mainpp.o \
./Core/Src/mecabot_hardware.o \
./Core/Src/pid.o \
./Core/Src/stm32f1xx_hal_msp.o \
./Core/Src/stm32f1xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f1xx.o \
./Core/Src/time.o 

CPP_DEPS += \
./Core/Src/IIR_filter.d \
./Core/Src/duration.d \
./Core/Src/mainpp.d \
./Core/Src/mecabot_hardware.d \
./Core/Src/time.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.cpp Core/Src/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m3 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xE -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/ADMIN/Desktop/DATN/Mecabot_Firmware/examples/../components/common/include" -I"C:/Users/ADMIN/Desktop/DATN/Mecabot_Firmware/examples/../components/dc_motor/include" -I"C:/Users/ADMIN/Desktop/DATN/Mecabot_Firmware/examples/../components/encoder/include" -I"C:/Users/ADMIN/Desktop/DATN/Mecabot_Firmware/examples/../components/filter/include" -I"C:/Users/ADMIN/Desktop/DATN/Mecabot_Firmware/examples/../components/pid/include" -I"C:/Users/ADMIN/Desktop/DATN/Mecabot_Firmware/examples/../components/rosserial/include" -I"C:/Users/ADMIN/Desktop/DATN/Mecabot_Firmware/examples/../components/mpu6050/include" -I"C:/Users/ADMIN/Desktop/DATN/Mecabot_Firmware/examples/../components/madgwick_filter/include" -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xE -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/ADMIN/Desktop/DATN/Mecabot_Firmware/examples/../components/common/include" -I"C:/Users/ADMIN/Desktop/DATN/Mecabot_Firmware/examples/../components/dc_motor/include" -I"C:/Users/ADMIN/Desktop/DATN/Mecabot_Firmware/examples/../components/encoder/include" -I"C:/Users/ADMIN/Desktop/DATN/Mecabot_Firmware/examples/../components/filter/include" -I"C:/Users/ADMIN/Desktop/DATN/Mecabot_Firmware/examples/../components/pid/include" -I"C:/Users/ADMIN/Desktop/DATN/Mecabot_Firmware/examples/../components/rosserial/include" -I"C:/Users/ADMIN/Desktop/DATN/Mecabot_Firmware/examples/../components/mpu6050/include" -I"C:/Users/ADMIN/Desktop/DATN/Mecabot_Firmware/examples/../components/madgwick_filter/include" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Core/Src/duration.o: C:/Users/ADMIN/Desktop/DATN/Mecabot_Firmware/components/rosserial/duration.cpp Core/Src/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m3 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xE -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/ADMIN/Desktop/DATN/Mecabot_Firmware/examples/../components/common/include" -I"C:/Users/ADMIN/Desktop/DATN/Mecabot_Firmware/examples/../components/dc_motor/include" -I"C:/Users/ADMIN/Desktop/DATN/Mecabot_Firmware/examples/../components/encoder/include" -I"C:/Users/ADMIN/Desktop/DATN/Mecabot_Firmware/examples/../components/filter/include" -I"C:/Users/ADMIN/Desktop/DATN/Mecabot_Firmware/examples/../components/pid/include" -I"C:/Users/ADMIN/Desktop/DATN/Mecabot_Firmware/examples/../components/rosserial/include" -I"C:/Users/ADMIN/Desktop/DATN/Mecabot_Firmware/examples/../components/mpu6050/include" -I"C:/Users/ADMIN/Desktop/DATN/Mecabot_Firmware/examples/../components/madgwick_filter/include" -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Core/Src/time.o: C:/Users/ADMIN/Desktop/DATN/Mecabot_Firmware/components/rosserial/time.cpp Core/Src/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m3 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xE -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/ADMIN/Desktop/DATN/Mecabot_Firmware/examples/../components/common/include" -I"C:/Users/ADMIN/Desktop/DATN/Mecabot_Firmware/examples/../components/dc_motor/include" -I"C:/Users/ADMIN/Desktop/DATN/Mecabot_Firmware/examples/../components/encoder/include" -I"C:/Users/ADMIN/Desktop/DATN/Mecabot_Firmware/examples/../components/filter/include" -I"C:/Users/ADMIN/Desktop/DATN/Mecabot_Firmware/examples/../components/pid/include" -I"C:/Users/ADMIN/Desktop/DATN/Mecabot_Firmware/examples/../components/rosserial/include" -I"C:/Users/ADMIN/Desktop/DATN/Mecabot_Firmware/examples/../components/mpu6050/include" -I"C:/Users/ADMIN/Desktop/DATN/Mecabot_Firmware/examples/../components/madgwick_filter/include" -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/IIR_filter.cyclo ./Core/Src/IIR_filter.d ./Core/Src/IIR_filter.o ./Core/Src/IIR_filter.su ./Core/Src/dc_motor.cyclo ./Core/Src/dc_motor.d ./Core/Src/dc_motor.o ./Core/Src/dc_motor.su ./Core/Src/duration.cyclo ./Core/Src/duration.d ./Core/Src/duration.o ./Core/Src/duration.su ./Core/Src/encoder.cyclo ./Core/Src/encoder.d ./Core/Src/encoder.o ./Core/Src/encoder.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/mainpp.cyclo ./Core/Src/mainpp.d ./Core/Src/mainpp.o ./Core/Src/mainpp.su ./Core/Src/mecabot_hardware.cyclo ./Core/Src/mecabot_hardware.d ./Core/Src/mecabot_hardware.o ./Core/Src/mecabot_hardware.su ./Core/Src/pid.cyclo ./Core/Src/pid.d ./Core/Src/pid.o ./Core/Src/pid.su ./Core/Src/stm32f1xx_hal_msp.cyclo ./Core/Src/stm32f1xx_hal_msp.d ./Core/Src/stm32f1xx_hal_msp.o ./Core/Src/stm32f1xx_hal_msp.su ./Core/Src/stm32f1xx_it.cyclo ./Core/Src/stm32f1xx_it.d ./Core/Src/stm32f1xx_it.o ./Core/Src/stm32f1xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f1xx.cyclo ./Core/Src/system_stm32f1xx.d ./Core/Src/system_stm32f1xx.o ./Core/Src/system_stm32f1xx.su ./Core/Src/time.cyclo ./Core/Src/time.d ./Core/Src/time.o ./Core/Src/time.su

.PHONY: clean-Core-2f-Src

