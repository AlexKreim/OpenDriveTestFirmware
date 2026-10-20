################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/esc_app.c \
../Core/Src/esc_flash.c \
../Core/Src/esc_foc.c \
../Core/Src/esc_hw.c \
../Core/Src/esc_motor.c \
../Core/Src/esc_params.c \
../Core/Src/main.c \
../Core/Src/stm32g4xx_hal_msp.c \
../Core/Src/stm32g4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32g4xx.c 

OBJS += \
./Core/Src/esc_app.o \
./Core/Src/esc_flash.o \
./Core/Src/esc_foc.o \
./Core/Src/esc_hw.o \
./Core/Src/esc_motor.o \
./Core/Src/esc_params.o \
./Core/Src/main.o \
./Core/Src/stm32g4xx_hal_msp.o \
./Core/Src/stm32g4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32g4xx.o 

C_DEPS += \
./Core/Src/esc_app.d \
./Core/Src/esc_flash.d \
./Core/Src/esc_foc.d \
./Core/Src/esc_hw.d \
./Core/Src/esc_motor.d \
./Core/Src/esc_params.d \
./Core/Src/main.d \
./Core/Src/stm32g4xx_hal_msp.d \
./Core/Src/stm32g4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32g4xx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G474xx -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -I../USB_Device/App -I../USB_Device/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -O2 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/esc_app.cyclo ./Core/Src/esc_app.d ./Core/Src/esc_app.o ./Core/Src/esc_app.su ./Core/Src/esc_flash.cyclo ./Core/Src/esc_flash.d ./Core/Src/esc_flash.o ./Core/Src/esc_flash.su ./Core/Src/esc_foc.cyclo ./Core/Src/esc_foc.d ./Core/Src/esc_foc.o ./Core/Src/esc_foc.su ./Core/Src/esc_hw.cyclo ./Core/Src/esc_hw.d ./Core/Src/esc_hw.o ./Core/Src/esc_hw.su ./Core/Src/esc_motor.cyclo ./Core/Src/esc_motor.d ./Core/Src/esc_motor.o ./Core/Src/esc_motor.su ./Core/Src/esc_params.cyclo ./Core/Src/esc_params.d ./Core/Src/esc_params.o ./Core/Src/esc_params.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/stm32g4xx_hal_msp.cyclo ./Core/Src/stm32g4xx_hal_msp.d ./Core/Src/stm32g4xx_hal_msp.o ./Core/Src/stm32g4xx_hal_msp.su ./Core/Src/stm32g4xx_it.cyclo ./Core/Src/stm32g4xx_it.d ./Core/Src/stm32g4xx_it.o ./Core/Src/stm32g4xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32g4xx.cyclo ./Core/Src/system_stm32g4xx.d ./Core/Src/system_stm32g4xx.o ./Core/Src/system_stm32g4xx.su

.PHONY: clean-Core-2f-Src

