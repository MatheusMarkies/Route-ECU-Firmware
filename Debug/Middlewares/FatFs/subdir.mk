################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
C:/Users/Matheus\ Markies/STM32Cube/Repository/STM32Cube_FW_H7_V1.12.1/Middlewares/Third_Party/FatFs/src/diskio.c \
C:/Users/Matheus\ Markies/STM32Cube/Repository/STM32Cube_FW_H7_V1.12.1/Middlewares/Third_Party/FatFs/src/ff.c \
C:/Users/Matheus\ Markies/STM32Cube/Repository/STM32Cube_FW_H7_V1.12.1/Middlewares/Third_Party/FatFs/src/ff_gen_drv.c \
C:/Users/Matheus\ Markies/STM32Cube/Repository/STM32Cube_FW_H7_V1.12.1/Middlewares/Third_Party/FatFs/src/option/syscall.c 

OBJS += \
./Middlewares/FatFs/diskio.o \
./Middlewares/FatFs/ff.o \
./Middlewares/FatFs/ff_gen_drv.o \
./Middlewares/FatFs/syscall.o 

C_DEPS += \
./Middlewares/FatFs/diskio.d \
./Middlewares/FatFs/ff.d \
./Middlewares/FatFs/ff_gen_drv.d \
./Middlewares/FatFs/syscall.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/FatFs/diskio.o: C:/Users/Matheus\ Markies/STM32Cube/Repository/STM32Cube_FW_H7_V1.12.1/Middlewares/Third_Party/FatFs/src/diskio.c Middlewares/FatFs/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_PWR_LDO_SUPPLY -DUSE_HAL_DRIVER -DSTM32H743xx -c -I../Core/Inc -I"C:/Users/Matheus Markies/STM32Cube/Repository/STM32Cube_FW_H7_V1.12.1/Drivers/STM32H7xx_HAL_Driver/Inc" -I"C:/Users/Matheus Markies/STM32Cube/Repository/STM32Cube_FW_H7_V1.12.1/Drivers/STM32H7xx_HAL_Driver/Inc/Legacy" -I"C:/Users/Matheus Markies/STM32Cube/Repository/STM32Cube_FW_H7_V1.12.1/Drivers/CMSIS/Device/ST/STM32H7xx/Include" -I"C:/Users/Matheus Markies/STM32Cube/Repository/STM32Cube_FW_H7_V1.12.1/Drivers/CMSIS/Include" -I../FATFS/Target -I../FATFS/App -I"C:/Users/Matheus Markies/STM32Cube/Repository/STM32Cube_FW_H7_V1.12.1/Middlewares/Third_Party/FreeRTOS/Source/include" -I"C:/Users/Matheus Markies/STM32Cube/Repository/STM32Cube_FW_H7_V1.12.1/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS" -I"C:/Users/Matheus Markies/STM32Cube/Repository/STM32Cube_FW_H7_V1.12.1/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F" -I"C:/Users/Matheus Markies/STM32Cube/Repository/STM32Cube_FW_H7_V1.12.1/Middlewares/Third_Party/FatFs/src" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"Middlewares/FatFs/diskio.d" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
Middlewares/FatFs/ff.o: C:/Users/Matheus\ Markies/STM32Cube/Repository/STM32Cube_FW_H7_V1.12.1/Middlewares/Third_Party/FatFs/src/ff.c Middlewares/FatFs/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_PWR_LDO_SUPPLY -DUSE_HAL_DRIVER -DSTM32H743xx -c -I../Core/Inc -I"C:/Users/Matheus Markies/STM32Cube/Repository/STM32Cube_FW_H7_V1.12.1/Drivers/STM32H7xx_HAL_Driver/Inc" -I"C:/Users/Matheus Markies/STM32Cube/Repository/STM32Cube_FW_H7_V1.12.1/Drivers/STM32H7xx_HAL_Driver/Inc/Legacy" -I"C:/Users/Matheus Markies/STM32Cube/Repository/STM32Cube_FW_H7_V1.12.1/Drivers/CMSIS/Device/ST/STM32H7xx/Include" -I"C:/Users/Matheus Markies/STM32Cube/Repository/STM32Cube_FW_H7_V1.12.1/Drivers/CMSIS/Include" -I../FATFS/Target -I../FATFS/App -I"C:/Users/Matheus Markies/STM32Cube/Repository/STM32Cube_FW_H7_V1.12.1/Middlewares/Third_Party/FreeRTOS/Source/include" -I"C:/Users/Matheus Markies/STM32Cube/Repository/STM32Cube_FW_H7_V1.12.1/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS" -I"C:/Users/Matheus Markies/STM32Cube/Repository/STM32Cube_FW_H7_V1.12.1/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F" -I"C:/Users/Matheus Markies/STM32Cube/Repository/STM32Cube_FW_H7_V1.12.1/Middlewares/Third_Party/FatFs/src" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"Middlewares/FatFs/ff.d" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
Middlewares/FatFs/ff_gen_drv.o: C:/Users/Matheus\ Markies/STM32Cube/Repository/STM32Cube_FW_H7_V1.12.1/Middlewares/Third_Party/FatFs/src/ff_gen_drv.c Middlewares/FatFs/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_PWR_LDO_SUPPLY -DUSE_HAL_DRIVER -DSTM32H743xx -c -I../Core/Inc -I"C:/Users/Matheus Markies/STM32Cube/Repository/STM32Cube_FW_H7_V1.12.1/Drivers/STM32H7xx_HAL_Driver/Inc" -I"C:/Users/Matheus Markies/STM32Cube/Repository/STM32Cube_FW_H7_V1.12.1/Drivers/STM32H7xx_HAL_Driver/Inc/Legacy" -I"C:/Users/Matheus Markies/STM32Cube/Repository/STM32Cube_FW_H7_V1.12.1/Drivers/CMSIS/Device/ST/STM32H7xx/Include" -I"C:/Users/Matheus Markies/STM32Cube/Repository/STM32Cube_FW_H7_V1.12.1/Drivers/CMSIS/Include" -I../FATFS/Target -I../FATFS/App -I"C:/Users/Matheus Markies/STM32Cube/Repository/STM32Cube_FW_H7_V1.12.1/Middlewares/Third_Party/FreeRTOS/Source/include" -I"C:/Users/Matheus Markies/STM32Cube/Repository/STM32Cube_FW_H7_V1.12.1/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS" -I"C:/Users/Matheus Markies/STM32Cube/Repository/STM32Cube_FW_H7_V1.12.1/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F" -I"C:/Users/Matheus Markies/STM32Cube/Repository/STM32Cube_FW_H7_V1.12.1/Middlewares/Third_Party/FatFs/src" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"Middlewares/FatFs/ff_gen_drv.d" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
Middlewares/FatFs/syscall.o: C:/Users/Matheus\ Markies/STM32Cube/Repository/STM32Cube_FW_H7_V1.12.1/Middlewares/Third_Party/FatFs/src/option/syscall.c Middlewares/FatFs/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_PWR_LDO_SUPPLY -DUSE_HAL_DRIVER -DSTM32H743xx -c -I../Core/Inc -I"C:/Users/Matheus Markies/STM32Cube/Repository/STM32Cube_FW_H7_V1.12.1/Drivers/STM32H7xx_HAL_Driver/Inc" -I"C:/Users/Matheus Markies/STM32Cube/Repository/STM32Cube_FW_H7_V1.12.1/Drivers/STM32H7xx_HAL_Driver/Inc/Legacy" -I"C:/Users/Matheus Markies/STM32Cube/Repository/STM32Cube_FW_H7_V1.12.1/Drivers/CMSIS/Device/ST/STM32H7xx/Include" -I"C:/Users/Matheus Markies/STM32Cube/Repository/STM32Cube_FW_H7_V1.12.1/Drivers/CMSIS/Include" -I../FATFS/Target -I../FATFS/App -I"C:/Users/Matheus Markies/STM32Cube/Repository/STM32Cube_FW_H7_V1.12.1/Middlewares/Third_Party/FreeRTOS/Source/include" -I"C:/Users/Matheus Markies/STM32Cube/Repository/STM32Cube_FW_H7_V1.12.1/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS" -I"C:/Users/Matheus Markies/STM32Cube/Repository/STM32Cube_FW_H7_V1.12.1/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F" -I"C:/Users/Matheus Markies/STM32Cube/Repository/STM32Cube_FW_H7_V1.12.1/Middlewares/Third_Party/FatFs/src" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"Middlewares/FatFs/syscall.d" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Middlewares-2f-FatFs

clean-Middlewares-2f-FatFs:
	-$(RM) ./Middlewares/FatFs/diskio.cyclo ./Middlewares/FatFs/diskio.d ./Middlewares/FatFs/diskio.o ./Middlewares/FatFs/diskio.su ./Middlewares/FatFs/ff.cyclo ./Middlewares/FatFs/ff.d ./Middlewares/FatFs/ff.o ./Middlewares/FatFs/ff.su ./Middlewares/FatFs/ff_gen_drv.cyclo ./Middlewares/FatFs/ff_gen_drv.d ./Middlewares/FatFs/ff_gen_drv.o ./Middlewares/FatFs/ff_gen_drv.su ./Middlewares/FatFs/syscall.cyclo ./Middlewares/FatFs/syscall.d ./Middlewares/FatFs/syscall.o ./Middlewares/FatFs/syscall.su

.PHONY: clean-Middlewares-2f-FatFs

