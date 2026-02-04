################################################################################
# 自动生成的文件。不要编辑！
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# 将这些工具调用的输入和输出添加到构建变量 
C_SRCS += \
../mycode/Encoder.c \
../mycode/KEY.c \
../mycode/LED.c \
../mycode/Motor.c \
../mycode/OLED.c \
../mycode/PID.c \
../mycode/Servo.c \
../mycode/USART.c \
../mycode/jy62.c 

OBJS += \
./mycode/Encoder.o \
./mycode/KEY.o \
./mycode/LED.o \
./mycode/Motor.o \
./mycode/OLED.o \
./mycode/PID.o \
./mycode/Servo.o \
./mycode/USART.o \
./mycode/jy62.o 

C_DEPS += \
./mycode/Encoder.d \
./mycode/KEY.d \
./mycode/LED.d \
./mycode/Motor.d \
./mycode/OLED.d \
./mycode/PID.d \
./mycode/Servo.d \
./mycode/USART.d \
./mycode/jy62.d 


# 每个子目录必须为构建它所贡献的源提供规则
mycode/%.o mycode/%.su mycode/%.cyclo: ../mycode/%.c mycode/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xE -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-mycode

clean-mycode:
	-$(RM) ./mycode/Encoder.cyclo ./mycode/Encoder.d ./mycode/Encoder.o ./mycode/Encoder.su ./mycode/KEY.cyclo ./mycode/KEY.d ./mycode/KEY.o ./mycode/KEY.su ./mycode/LED.cyclo ./mycode/LED.d ./mycode/LED.o ./mycode/LED.su ./mycode/Motor.cyclo ./mycode/Motor.d ./mycode/Motor.o ./mycode/Motor.su ./mycode/OLED.cyclo ./mycode/OLED.d ./mycode/OLED.o ./mycode/OLED.su ./mycode/PID.cyclo ./mycode/PID.d ./mycode/PID.o ./mycode/PID.su ./mycode/Servo.cyclo ./mycode/Servo.d ./mycode/Servo.o ./mycode/Servo.su ./mycode/USART.cyclo ./mycode/USART.d ./mycode/USART.o ./mycode/USART.su ./mycode/jy62.cyclo ./mycode/jy62.d ./mycode/jy62.o ./mycode/jy62.su

.PHONY: clean-mycode

