################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../USB_DEVICE/Target/usbd_conf.c 

C_DEPS += \
./USB_DEVICE/Target/usbd_conf.d 

OBJS += \
./USB_DEVICE/Target/usbd_conf.o 


# Each subdirectory must supply rules for building sources it contributes
USB_DEVICE/Target/%.o USB_DEVICE/Target/%.su: ../USB_DEVICE/Target/%.c USB_DEVICE/Target/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F042x6 -c -I../Core/Inc -I../USB_DEVICE/App -I../USB_DEVICE/Target -IC:/Users/Nand/STM32Cube/Repository/STM32Cube_FW_F0_V1.11.5/Drivers/STM32F0xx_HAL_Driver/Inc -IC:/Users/Nand/STM32Cube/Repository/STM32Cube_FW_F0_V1.11.5/Drivers/STM32F0xx_HAL_Driver/Inc/Legacy -IC:/Users/Nand/STM32Cube/Repository/STM32Cube_FW_F0_V1.11.5/Middlewares/ST/STM32_USB_Device_Library/Core/Inc -IC:/Users/Nand/STM32Cube/Repository/STM32Cube_FW_F0_V1.11.5/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -IC:/Users/Nand/STM32Cube/Repository/STM32Cube_FW_F0_V1.11.5/Drivers/CMSIS/Device/ST/STM32F0xx/Include -IC:/Users/Nand/STM32Cube/Repository/STM32Cube_FW_F0_V1.11.5/Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-USB_DEVICE-2f-Target

clean-USB_DEVICE-2f-Target:
	-$(RM) ./USB_DEVICE/Target/usbd_conf.d ./USB_DEVICE/Target/usbd_conf.o ./USB_DEVICE/Target/usbd_conf.su

.PHONY: clean-USB_DEVICE-2f-Target

