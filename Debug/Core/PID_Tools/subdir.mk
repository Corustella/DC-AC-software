################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/PID_Tools/float_tools.c 

OBJS += \
./Core/PID_Tools/float_tools.o 

C_DEPS += \
./Core/PID_Tools/float_tools.d 


# Each subdirectory must supply rules for building sources it contributes
Core/PID_Tools/%.o Core/PID_Tools/%.su Core/PID_Tools/%.cyclo: ../Core/PID_Tools/%.c Core/PID_Tools/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -DARM_MATH_CM4 -D__FPU_PRESENT=1 -D__TARGET_FPU_VFP -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/ST/ARM/DSP/Inc -I../Middlewares/ST/ARM/DSP/PrivateInclude -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-PID_Tools

clean-Core-2f-PID_Tools:
	-$(RM) ./Core/PID_Tools/float_tools.cyclo ./Core/PID_Tools/float_tools.d ./Core/PID_Tools/float_tools.o ./Core/PID_Tools/float_tools.su

.PHONY: clean-Core-2f-PID_Tools

