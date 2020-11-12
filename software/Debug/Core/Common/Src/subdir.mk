################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Common/Src/AMS_CAN_Messages.c \
../Core/Common/Src/BMS_CAN_Messages.c \
../Core/Common/Src/CC_CAN_Messages.c \
../Core/Common/Src/CC_CAN_Wrapper.c \
../Core/Common/Src/FSM.c \
../Core/Common/Src/PDM_CAN_Messages.c \
../Core/Common/Src/QUTMS_can.c \
../Core/Common/Src/SHDN_BSPD_CAN_Messages.c \
../Core/Common/Src/SHDN_CAN_Messages.c \
../Core/Common/Src/SHDN_IMD_CAN_Messages.c \
../Core/Common/Src/Util.c 

OBJS += \
./Core/Common/Src/AMS_CAN_Messages.o \
./Core/Common/Src/BMS_CAN_Messages.o \
./Core/Common/Src/CC_CAN_Messages.o \
./Core/Common/Src/CC_CAN_Wrapper.o \
./Core/Common/Src/FSM.o \
./Core/Common/Src/PDM_CAN_Messages.o \
./Core/Common/Src/QUTMS_can.o \
./Core/Common/Src/SHDN_BSPD_CAN_Messages.o \
./Core/Common/Src/SHDN_CAN_Messages.o \
./Core/Common/Src/SHDN_IMD_CAN_Messages.o \
./Core/Common/Src/Util.o 

C_DEPS += \
./Core/Common/Src/AMS_CAN_Messages.d \
./Core/Common/Src/BMS_CAN_Messages.d \
./Core/Common/Src/CC_CAN_Messages.d \
./Core/Common/Src/CC_CAN_Wrapper.d \
./Core/Common/Src/FSM.d \
./Core/Common/Src/PDM_CAN_Messages.d \
./Core/Common/Src/QUTMS_can.d \
./Core/Common/Src/SHDN_BSPD_CAN_Messages.d \
./Core/Common/Src/SHDN_CAN_Messages.d \
./Core/Common/Src/SHDN_IMD_CAN_Messages.d \
./Core/Common/Src/Util.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Common/Src/AMS_CAN_Messages.o: ../Core/Common/Src/AMS_CAN_Messages.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DSTM32F765xx -DQUTMS_CAN_SHDN -DQUTMS_CAN_CC -DDEBUG -DQUTMS_FSM -DQUTMS_CAN_SHDN_BSPD -DUSE_HAL_DRIVER -DQUTMS_CAN_SHDN_IMD -DQUTMS_UTIL -DQUTMS_CAN_PDM -DQUTMS_CAN_AMS -DQUTMS_CAN_WRAPPER_CC -c -I../Core/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1 -I"/home/alistair/Dev/repos/QUTMS/QUTMS_ChassisController/software/Core/Common/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Common/Src/AMS_CAN_Messages.d" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/Common/Src/BMS_CAN_Messages.o: ../Core/Common/Src/BMS_CAN_Messages.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DSTM32F765xx -DQUTMS_CAN_SHDN -DQUTMS_CAN_CC -DDEBUG -DQUTMS_FSM -DQUTMS_CAN_SHDN_BSPD -DUSE_HAL_DRIVER -DQUTMS_CAN_SHDN_IMD -DQUTMS_UTIL -DQUTMS_CAN_PDM -DQUTMS_CAN_AMS -DQUTMS_CAN_WRAPPER_CC -c -I../Core/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1 -I"/home/alistair/Dev/repos/QUTMS/QUTMS_ChassisController/software/Core/Common/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Common/Src/BMS_CAN_Messages.d" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/Common/Src/CC_CAN_Messages.o: ../Core/Common/Src/CC_CAN_Messages.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DSTM32F765xx -DQUTMS_CAN_SHDN -DQUTMS_CAN_CC -DDEBUG -DQUTMS_FSM -DQUTMS_CAN_SHDN_BSPD -DUSE_HAL_DRIVER -DQUTMS_CAN_SHDN_IMD -DQUTMS_UTIL -DQUTMS_CAN_PDM -DQUTMS_CAN_AMS -DQUTMS_CAN_WRAPPER_CC -c -I../Core/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1 -I"/home/alistair/Dev/repos/QUTMS/QUTMS_ChassisController/software/Core/Common/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Common/Src/CC_CAN_Messages.d" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/Common/Src/CC_CAN_Wrapper.o: ../Core/Common/Src/CC_CAN_Wrapper.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DSTM32F765xx -DQUTMS_CAN_SHDN -DQUTMS_CAN_CC -DDEBUG -DQUTMS_FSM -DQUTMS_CAN_SHDN_BSPD -DUSE_HAL_DRIVER -DQUTMS_CAN_SHDN_IMD -DQUTMS_UTIL -DQUTMS_CAN_PDM -DQUTMS_CAN_AMS -DQUTMS_CAN_WRAPPER_CC -c -I../Core/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1 -I"/home/alistair/Dev/repos/QUTMS/QUTMS_ChassisController/software/Core/Common/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Common/Src/CC_CAN_Wrapper.d" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/Common/Src/FSM.o: ../Core/Common/Src/FSM.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DSTM32F765xx -DQUTMS_CAN_SHDN -DQUTMS_CAN_CC -DDEBUG -DQUTMS_FSM -DQUTMS_CAN_SHDN_BSPD -DUSE_HAL_DRIVER -DQUTMS_CAN_SHDN_IMD -DQUTMS_UTIL -DQUTMS_CAN_PDM -DQUTMS_CAN_AMS -DQUTMS_CAN_WRAPPER_CC -c -I../Core/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1 -I"/home/alistair/Dev/repos/QUTMS/QUTMS_ChassisController/software/Core/Common/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Common/Src/FSM.d" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/Common/Src/PDM_CAN_Messages.o: ../Core/Common/Src/PDM_CAN_Messages.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DSTM32F765xx -DQUTMS_CAN_SHDN -DQUTMS_CAN_CC -DDEBUG -DQUTMS_FSM -DQUTMS_CAN_SHDN_BSPD -DUSE_HAL_DRIVER -DQUTMS_CAN_SHDN_IMD -DQUTMS_UTIL -DQUTMS_CAN_PDM -DQUTMS_CAN_AMS -DQUTMS_CAN_WRAPPER_CC -c -I../Core/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1 -I"/home/alistair/Dev/repos/QUTMS/QUTMS_ChassisController/software/Core/Common/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Common/Src/PDM_CAN_Messages.d" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/Common/Src/QUTMS_can.o: ../Core/Common/Src/QUTMS_can.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DSTM32F765xx -DQUTMS_CAN_SHDN -DQUTMS_CAN_CC -DDEBUG -DQUTMS_FSM -DQUTMS_CAN_SHDN_BSPD -DUSE_HAL_DRIVER -DQUTMS_CAN_SHDN_IMD -DQUTMS_UTIL -DQUTMS_CAN_PDM -DQUTMS_CAN_AMS -DQUTMS_CAN_WRAPPER_CC -c -I../Core/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1 -I"/home/alistair/Dev/repos/QUTMS/QUTMS_ChassisController/software/Core/Common/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Common/Src/QUTMS_can.d" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/Common/Src/SHDN_BSPD_CAN_Messages.o: ../Core/Common/Src/SHDN_BSPD_CAN_Messages.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DSTM32F765xx -DQUTMS_CAN_SHDN -DQUTMS_CAN_CC -DDEBUG -DQUTMS_FSM -DQUTMS_CAN_SHDN_BSPD -DUSE_HAL_DRIVER -DQUTMS_CAN_SHDN_IMD -DQUTMS_UTIL -DQUTMS_CAN_PDM -DQUTMS_CAN_AMS -DQUTMS_CAN_WRAPPER_CC -c -I../Core/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1 -I"/home/alistair/Dev/repos/QUTMS/QUTMS_ChassisController/software/Core/Common/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Common/Src/SHDN_BSPD_CAN_Messages.d" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/Common/Src/SHDN_CAN_Messages.o: ../Core/Common/Src/SHDN_CAN_Messages.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DSTM32F765xx -DQUTMS_CAN_SHDN -DQUTMS_CAN_CC -DDEBUG -DQUTMS_FSM -DQUTMS_CAN_SHDN_BSPD -DUSE_HAL_DRIVER -DQUTMS_CAN_SHDN_IMD -DQUTMS_UTIL -DQUTMS_CAN_PDM -DQUTMS_CAN_AMS -DQUTMS_CAN_WRAPPER_CC -c -I../Core/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1 -I"/home/alistair/Dev/repos/QUTMS/QUTMS_ChassisController/software/Core/Common/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Common/Src/SHDN_CAN_Messages.d" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/Common/Src/SHDN_IMD_CAN_Messages.o: ../Core/Common/Src/SHDN_IMD_CAN_Messages.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DSTM32F765xx -DQUTMS_CAN_SHDN -DQUTMS_CAN_CC -DDEBUG -DQUTMS_FSM -DQUTMS_CAN_SHDN_BSPD -DUSE_HAL_DRIVER -DQUTMS_CAN_SHDN_IMD -DQUTMS_UTIL -DQUTMS_CAN_PDM -DQUTMS_CAN_AMS -DQUTMS_CAN_WRAPPER_CC -c -I../Core/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1 -I"/home/alistair/Dev/repos/QUTMS/QUTMS_ChassisController/software/Core/Common/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Common/Src/SHDN_IMD_CAN_Messages.d" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/Common/Src/Util.o: ../Core/Common/Src/Util.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DSTM32F765xx -DQUTMS_CAN_SHDN -DQUTMS_CAN_CC -DDEBUG -DQUTMS_FSM -DQUTMS_CAN_SHDN_BSPD -DUSE_HAL_DRIVER -DQUTMS_CAN_SHDN_IMD -DQUTMS_UTIL -DQUTMS_CAN_PDM -DQUTMS_CAN_AMS -DQUTMS_CAN_WRAPPER_CC -c -I../Core/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1 -I"/home/alistair/Dev/repos/QUTMS/QUTMS_ChassisController/software/Core/Common/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Common/Src/Util.d" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

