################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Applications/rfid_app/rfid_app.c \
../Applications/rfid_app/rfid_command.c 

OBJS += \
./Applications/rfid_app/rfid_app.o \
./Applications/rfid_app/rfid_command.o 

C_DEPS += \
./Applications/rfid_app/rfid_app.d \
./Applications/rfid_app/rfid_command.d 


# Each subdirectory must supply rules for building sources it contributes
Applications/rfid_app/rfid_app.o: ../Applications/rfid_app/rfid_app.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F746xx -DDEBUG -c -I../Core/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1 -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -I../Drivers/led -I../Drivers/log_debug -I../Drivers/rfid -I../Applications/rfid_app -I../Applications/heart_beat_app -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Applications/rfid_app/rfid_app.d" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Applications/rfid_app/rfid_command.o: ../Applications/rfid_app/rfid_command.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F746xx -DDEBUG -c -I../Core/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1 -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -I../Drivers/led -I../Drivers/log_debug -I../Drivers/rfid -I../Applications/rfid_app -I../Applications/heart_beat_app -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Applications/rfid_app/rfid_command.d" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

