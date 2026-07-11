################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../my_drivers/pid_tools/float_tools.c 

OBJS += \
./my_drivers/pid_tools/float_tools.o 

C_DEPS += \
./my_drivers/pid_tools/float_tools.d 


# Each subdirectory must supply rules for building sources it contributes
my_drivers/pid_tools/%.o my_drivers/pid_tools/%.su my_drivers/pid_tools/%.cyclo: ../my_drivers/pid_tools/%.c my_drivers/pid_tools/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -DARM_MATH_CM4 -D__FPU_PRESENT=1 -D__TARGET_FPU_VFP -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/ST/ARM/DSP/Inc -I../Middlewares/ST/ARM/DSP/PrivateInclude -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-my_drivers-2f-pid_tools

clean-my_drivers-2f-pid_tools:
	-$(RM) ./my_drivers/pid_tools/float_tools.cyclo ./my_drivers/pid_tools/float_tools.d ./my_drivers/pid_tools/float_tools.o ./my_drivers/pid_tools/float_tools.su

.PHONY: clean-my_drivers-2f-pid_tools

