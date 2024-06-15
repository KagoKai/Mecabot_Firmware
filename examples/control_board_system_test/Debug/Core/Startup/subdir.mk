################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../Core/Startup/startup_stm32f103rctx.s 

S_DEPS += \
./Core/Startup/startup_stm32f103rctx.d 

OBJS += \
./Core/Startup/startup_stm32f103rctx.o 


# Each subdirectory must supply rules for building sources it contributes
Core/Startup/%.o: ../Core/Startup/%.s Core/Startup/subdir.mk
	arm-none-eabi-gcc -mcpu=cortex-m3 -g3 -DDEBUG -c -I"C:/Users/ADMIN/Desktop/DATN/Mecabot_Firmware/examples/../components/common/include" -I"C:/Users/ADMIN/Desktop/DATN/Mecabot_Firmware/examples/../components/dc_motor/include" -I"C:/Users/ADMIN/Desktop/DATN/Mecabot_Firmware/examples/../components/encoder/include" -I"C:/Users/ADMIN/Desktop/DATN/Mecabot_Firmware/examples/../components/filter/include" -I"C:/Users/ADMIN/Desktop/DATN/Mecabot_Firmware/examples/../components/pid/include" -I"C:/Users/ADMIN/Desktop/DATN/Mecabot_Firmware/examples/../components/rosserial/include" -I"C:/Users/ADMIN/Desktop/DATN/Mecabot_Firmware/examples/../components/mpu6050/include" -I"C:/Users/ADMIN/Desktop/DATN/Mecabot_Firmware/examples/../components/madgwick_filter/include" -x assembler-with-cpp -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@" "$<"

clean: clean-Core-2f-Startup

clean-Core-2f-Startup:
	-$(RM) ./Core/Startup/startup_stm32f103rctx.d ./Core/Startup/startup_stm32f103rctx.o

.PHONY: clean-Core-2f-Startup

