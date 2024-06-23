################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
C:/Users/ADMIN/Desktop/DATN/Mecabot_Firmware/components/mpu6050_dmp/I2Cdev.cpp \
C:/Users/ADMIN/Desktop/DATN/Mecabot_Firmware/components/mpu6050_dmp/MPU6050.cpp \
C:/Users/ADMIN/Desktop/DATN/Mecabot_Firmware/components/mpu6050_dmp/MPU6050_6Axis_MotionApps20.cpp 

OBJS += \
./Core/Src/mpu6050_dmp/I2Cdev.o \
./Core/Src/mpu6050_dmp/MPU6050.o \
./Core/Src/mpu6050_dmp/MPU6050_6Axis_MotionApps20.o 

CPP_DEPS += \
./Core/Src/mpu6050_dmp/I2Cdev.d \
./Core/Src/mpu6050_dmp/MPU6050.d \
./Core/Src/mpu6050_dmp/MPU6050_6Axis_MotionApps20.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/mpu6050_dmp/I2Cdev.o: C:/Users/ADMIN/Desktop/DATN/Mecabot_Firmware/components/mpu6050_dmp/I2Cdev.cpp Core/Src/mpu6050_dmp/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m3 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xE -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/ADMIN/Desktop/DATN/Mecabot_Firmware/components/mpu6050_dmp/include" -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Core/Src/mpu6050_dmp/MPU6050.o: C:/Users/ADMIN/Desktop/DATN/Mecabot_Firmware/components/mpu6050_dmp/MPU6050.cpp Core/Src/mpu6050_dmp/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m3 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xE -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/ADMIN/Desktop/DATN/Mecabot_Firmware/components/mpu6050_dmp/include" -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Core/Src/mpu6050_dmp/MPU6050_6Axis_MotionApps20.o: C:/Users/ADMIN/Desktop/DATN/Mecabot_Firmware/components/mpu6050_dmp/MPU6050_6Axis_MotionApps20.cpp Core/Src/mpu6050_dmp/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m3 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xE -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/ADMIN/Desktop/DATN/Mecabot_Firmware/components/mpu6050_dmp/include" -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-mpu6050_dmp

clean-Core-2f-Src-2f-mpu6050_dmp:
	-$(RM) ./Core/Src/mpu6050_dmp/I2Cdev.cyclo ./Core/Src/mpu6050_dmp/I2Cdev.d ./Core/Src/mpu6050_dmp/I2Cdev.o ./Core/Src/mpu6050_dmp/I2Cdev.su ./Core/Src/mpu6050_dmp/MPU6050.cyclo ./Core/Src/mpu6050_dmp/MPU6050.d ./Core/Src/mpu6050_dmp/MPU6050.o ./Core/Src/mpu6050_dmp/MPU6050.su ./Core/Src/mpu6050_dmp/MPU6050_6Axis_MotionApps20.cyclo ./Core/Src/mpu6050_dmp/MPU6050_6Axis_MotionApps20.d ./Core/Src/mpu6050_dmp/MPU6050_6Axis_MotionApps20.o ./Core/Src/mpu6050_dmp/MPU6050_6Axis_MotionApps20.su

.PHONY: clean-Core-2f-Src-2f-mpu6050_dmp

