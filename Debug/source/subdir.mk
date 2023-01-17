################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../source/FlexCanNonBlocking.c \
../source/semihost_hardfault.c 

C_DEPS += \
./source/FlexCanNonBlocking.d \
./source/semihost_hardfault.d 

OBJS += \
./source/FlexCanNonBlocking.o \
./source/semihost_hardfault.o 


# Each subdirectory must supply rules for building sources it contributes
source/%.o: ../source/%.c source/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: MCU C Compiler'
	arm-none-eabi-gcc -D__REDLIB__ -DCPU_MK64FN1M0VLL12 -DCPU_MK64FN1M0VLL12_cm4 -DSDK_OS_BAREMETAL -DSDK_DEBUGCONSOLE=1 -DCR_INTEGER_PRINTF -DPRINTF_FLOAT_ENABLE=0 -DSERIAL_PORT_TYPE_UART=1 -D__MCUXPRESSO -D__USE_CMSIS -DDEBUG -I"C:\Users\Lauta\Documents\MCUXpressoIDE_11.6.1_8255\workspace\FlexCanNonBlocking\board" -I"C:\Users\Lauta\Documents\MCUXpressoIDE_11.6.1_8255\workspace\FlexCanNonBlocking\source" -I"C:\Users\Lauta\Documents\MCUXpressoIDE_11.6.1_8255\workspace\FlexCanNonBlocking\drivers" -I"C:\Users\Lauta\Documents\MCUXpressoIDE_11.6.1_8255\workspace\FlexCanNonBlocking\device" -I"C:\Users\Lauta\Documents\MCUXpressoIDE_11.6.1_8255\workspace\FlexCanNonBlocking\CMSIS" -I"C:\Users\Lauta\Documents\MCUXpressoIDE_11.6.1_8255\workspace\FlexCanNonBlocking\utilities" -I"C:\Users\Lauta\Documents\MCUXpressoIDE_11.6.1_8255\workspace\FlexCanNonBlocking\component\serial_manager" -I"C:\Users\Lauta\Documents\MCUXpressoIDE_11.6.1_8255\workspace\FlexCanNonBlocking\component\uart" -I"C:\Users\Lauta\Documents\MCUXpressoIDE_11.6.1_8255\workspace\FlexCanNonBlocking\component\lists" -O0 -fno-common -g3 -Wall -c -ffunction-sections -fdata-sections -ffreestanding -fno-builtin -fmerge-constants -fmacro-prefix-map="$(<D)/"= -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -D__REDLIB__ -fstack-usage -specs=redlib.specs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


clean: clean-source

clean-source:
	-$(RM) ./source/FlexCanNonBlocking.d ./source/FlexCanNonBlocking.o ./source/semihost_hardfault.d ./source/semihost_hardfault.o

.PHONY: clean-source

