################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/hardware/canfd0.c \
../src/hardware/drv_uart.c \
../src/hardware/jiesuan.c 

C_DEPS += \
./src/hardware/canfd0.d \
./src/hardware/drv_uart.d \
./src/hardware/jiesuan.d 

CREF += \
vision_CPU1.cref 

OBJS += \
./src/hardware/canfd0.o \
./src/hardware/drv_uart.o \
./src/hardware/jiesuan.o 

MAP += \
vision_CPU1.map 


# Each subdirectory must supply rules for building sources it contributes
src/hardware/%.o: ../src/hardware/%.c
	@echo 'Building file: $<'
	$(file > $@.in,-mcpu=cortex-m33 -mthumb -mlittle-endian -mfloat-abi=hard -mfpu=fpv5-sp-d16 -Os -ffunction-sections -fdata-sections -fno-strict-aliasing -fmessage-length=0 -funsigned-char -Wunused -Wuninitialized -Wall -Wextra -Wmissing-declarations -Wconversion -Wpointer-arith -Wshadow -Waggregate-return -Wno-parentheses-equality -Wfloat-equal -g3 -std=c99 -fshort-enums -fno-unroll-loops -I"D:\\RA_R8P_Workspace_copy\\vision_CPU1\\ra_cfg\\fsp_cfg\\bsp" -I"." -I"D:\\RA_R8P_Workspace_copy\\vision_CPU1\\ra_gen" -I"D:\\RA_R8P_Workspace_copy\\vision_CPU1\\ra_cfg\\fsp_cfg" -I"D:\\RA_R8P_Workspace_copy\\vision_CPU1\\ra_cfg\\aws" -I"D:\\RA_R8P_Workspace_copy\\vision_CPU1\\ra\\arm\\CMSIS_6\\CMSIS\\Core\\Include" -I"D:\\RA_R8P_Workspace_copy\\vision_CPU1\\src" -I"D:\\RA_R8P_Workspace_copy\\vision_CPU1\\ra\\fsp\\inc" -I"D:\\RA_R8P_Workspace_copy\\vision_CPU1\\ra\\fsp\\inc\\api" -I"D:\\RA_R8P_Workspace_copy\\vision_CPU1\\ra\\fsp\\inc\\instances" -I"D:\\RA_R8P_Workspace_copy\\vision_CPU1\\ra\\fsp\\src\\rm_freertos_port" -I"D:\\RA_R8P_Workspace_copy\\vision_CPU1\\ra\\aws\\FreeRTOS\\FreeRTOS\\Source\\include" -D_RENESAS_RA_ -D_RA_CORE=CPU1 -D_RA_ORDINAL=2 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -x c "$<" -c -o "$@")
	@clang --target=arm-none-eabi @"$@.in"

