################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/Uart9_thread_entry.c \
../src/hal_warmstart.c 

C_DEPS += \
./src/Uart9_thread_entry.d \
./src/hal_warmstart.d 

CREF += \
vision_CPU0.cref 

OBJS += \
./src/Uart9_thread_entry.o \
./src/hal_warmstart.o 

MAP += \
vision_CPU0.map 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.c
	@echo 'Building file: $<'
	$(file > $@.in,-mcpu=cortex-m85 -mthumb -mlittle-endian -mfloat-abi=hard -Os -ffunction-sections -fdata-sections -fno-strict-aliasing -fmessage-length=0 -funsigned-char -Wunused -Wuninitialized -Wall -Wextra -Wmissing-declarations -Wconversion -Wpointer-arith -Wshadow -Waggregate-return -Wno-parentheses-equality -Wfloat-equal -g3 -std=c99 -flax-vector-conversions -fshort-enums -fno-unroll-loops -I"D:\\RA_R8P_Workspace_copy\\vision_CPU0\\ra_cfg\\fsp_cfg\\bsp" -I"." -I"D:\\RA_R8P_Workspace_copy\\vision_CPU0\\ra_gen" -I"D:\\RA_R8P_Workspace_copy\\vision_CPU0\\ra_cfg\\fsp_cfg" -I"D:\\RA_R8P_Workspace_copy\\vision_CPU0\\ra_cfg\\aws" -I"D:\\RA_R8P_Workspace_copy\\vision_CPU0\\ra\\arm\\CMSIS_6\\CMSIS\\Core\\Include" -I"D:\\RA_R8P_Workspace_copy\\vision_CPU0\\src" -I"D:\\RA_R8P_Workspace_copy\\vision_CPU0\\ra\\fsp\\inc" -I"D:\\RA_R8P_Workspace_copy\\vision_CPU0\\ra\\fsp\\inc\\api" -I"D:\\RA_R8P_Workspace_copy\\vision_CPU0\\ra\\fsp\\inc\\instances" -I"D:\\RA_R8P_Workspace_copy\\vision_CPU0\\ra\\fsp\\src\\rm_freertos_port" -I"D:\\RA_R8P_Workspace_copy\\vision_CPU0\\ra\\aws\\FreeRTOS\\FreeRTOS\\Source\\include" -D_RENESAS_RA_ -D_RA_CORE=CPU0 -D_RA_ORDINAL=1 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -x c "$<" -c -o "$@")
	@clang --target=arm-none-eabi @"$@.in"

