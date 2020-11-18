################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../Core/Startup/startup_stm32f765vitx.s 

OBJS += \
./Core/Startup/startup_stm32f765vitx.o 

S_DEPS += \
./Core/Startup/startup_stm32f765vitx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Startup/startup_stm32f765vitx.o: ../Core/Startup/startup_stm32f765vitx.s
	arm-none-eabi-gcc -mcpu=cortex-m7 -c -x assembler-with-cpp -MMD -MP -MF"Core/Startup/startup_stm32f765vitx.d" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@" "$<"

