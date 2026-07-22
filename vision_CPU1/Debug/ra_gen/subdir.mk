################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../ra_gen/Can_Thread.c \
../ra_gen/Uart9_thread.c \
../ra_gen/common_data.c \
../ra_gen/hal_data.c \
../ra_gen/main.c \
../ra_gen/pin_data.c \
../ra_gen/vector_data.c 

C_DEPS += \
./ra_gen/Can_Thread.d \
./ra_gen/Uart9_thread.d \
./ra_gen/common_data.d \
./ra_gen/hal_data.d \
./ra_gen/main.d \
./ra_gen/pin_data.d \
./ra_gen/vector_data.d 

CREF += \
vision_CPU1.cref 

OBJS += \
./ra_gen/Can_Thread.o \
./ra_gen/Uart9_thread.o \
./ra_gen/common_data.o \
./ra_gen/hal_data.o \
./ra_gen/main.o \
./ra_gen/pin_data.o \
./ra_gen/vector_data.o 

MAP += \
vision_CPU1.map 


# Each subdirectory must supply rules for building sources it contributes
ra_gen/%.o: ../ra_gen/%.c
	@echo 'Building file: $<'
	$(file > $@.in,-mcpu=cortex-m33 -mthumb -mlittle-endian -mfloat-abi=hard -mfpu=fpv5-sp-d16 -Os -ffunction-sections -fdata-sections -fno-strict-aliasing -fmessage-length=0 -funsigned-char -Wunused -Wuninitialized -Wall -Wextra -Wmissing-declarations -Wconversion -Wpointer-arith -Wshadow -Waggregate-return -Wno-parentheses-equality -Wfloat-equal -g3 -std=c99 -fshort-enums -fno-unroll-loops -I"D:\\RA_R8P_Workspace_copy\\vision_CPU1\\ra_cfg\\fsp_cfg\\bsp" -I"." -I"D:\\RA_R8P_Workspace_copy\\vision_CPU1\\ra_gen" -I"D:\\RA_R8P_Workspace_copy\\vision_CPU1\\ra_cfg\\fsp_cfg" -I"D:\\RA_R8P_Workspace_copy\\vision_CPU1\\ra_cfg\\aws" -I"D:\\RA_R8P_Workspace_copy\\vision_CPU1\\ra\\arm\\CMSIS_6\\CMSIS\\Core\\Include" -I"D:\\RA_R8P_Workspace_copy\\vision_CPU1\\src" -I"D:\\RA_R8P_Workspace_copy\\vision_CPU1\\ra\\fsp\\inc" -I"D:\\RA_R8P_Workspace_copy\\vision_CPU1\\ra\\fsp\\inc\\api" -I"D:\\RA_R8P_Workspace_copy\\vision_CPU1\\ra\\fsp\\inc\\instances" -I"D:\\RA_R8P_Workspace_copy\\vision_CPU1\\ra\\fsp\\src\\rm_freertos_port" -I"D:\\RA_R8P_Workspace_copy\\vision_CPU1\\ra\\aws\\FreeRTOS\\FreeRTOS\\Source\\include" -D_RENESAS_RA_ -D_RA_CORE=CPU1 -D_RA_ORDINAL=2 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -x c "$<" -c -o "$@")
	@clang --target=arm-none-eabi @"$@.in"

