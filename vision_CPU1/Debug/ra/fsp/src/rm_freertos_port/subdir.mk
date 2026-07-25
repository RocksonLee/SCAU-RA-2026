################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../ra/fsp/src/rm_freertos_port/port.c 

C_DEPS += \
./ra/fsp/src/rm_freertos_port/port.d 

CREF += \
vision_CPU1.cref 

OBJS += \
./ra/fsp/src/rm_freertos_port/port.o 

MAP += \
vision_CPU1.map 


# Each subdirectory must supply rules for building sources it contributes
ra/fsp/src/rm_freertos_port/%.o: ../ra/fsp/src/rm_freertos_port/%.c
	@echo 'Building file: $<'
	$(file > $@.in,-mcpu=cortex-m33 -mthumb -mlittle-endian -mfloat-abi=hard -mfpu=fpv5-sp-d16 -Os -ffunction-sections -fdata-sections -fno-strict-aliasing -fmessage-length=0 -funsigned-char -Wunused -Wuninitialized -Wall -Wextra -Wmissing-declarations -Wconversion -Wpointer-arith -Wshadow -Waggregate-return -Wno-parentheses-equality -Wfloat-equal -g3 -std=c99 -fshort-enums -fno-unroll-loops -I"D:\\e2studioworkspace\\SCAU-RA-2026\\vision_CPU1\\ra_cfg\\fsp_cfg\\bsp" -I"D:\\e2studioworkspace\\SCAU-RA-2026\\vision_CPU1\\ra\\lvgl\\lvgl" -I"." -I"D:\\e2studioworkspace\\SCAU-RA-2026\\vision_CPU1\\ra_gen" -I"D:\\e2studioworkspace\\SCAU-RA-2026\\vision_CPU1\\ra_cfg\\fsp_cfg" -I"D:\\e2studioworkspace\\SCAU-RA-2026\\vision_CPU1\\ra_cfg\\aws" -I"D:\\e2studioworkspace\\SCAU-RA-2026\\vision_CPU1\\ra\\arm\\CMSIS_6\\CMSIS\\Core\\Include" -I"D:\\e2studioworkspace\\SCAU-RA-2026\\vision_CPU1\\src" -I"D:\\e2studioworkspace\\SCAU-RA-2026\\vision_CPU1\\ra\\fsp\\inc" -I"D:\\e2studioworkspace\\SCAU-RA-2026\\vision_CPU1\\ra\\fsp\\inc\\api" -I"D:\\e2studioworkspace\\SCAU-RA-2026\\vision_CPU1\\ra\\fsp\\inc\\instances" -I"D:\\e2studioworkspace\\SCAU-RA-2026\\vision_CPU1\\ra\\fsp\\src\\rm_freertos_port" -I"D:\\e2studioworkspace\\SCAU-RA-2026\\vision_CPU1\\ra\\aws\\FreeRTOS\\FreeRTOS\\Source\\include" -D_RENESAS_RA_ -D_RA_CORE=CPU1 -D_RA_ORDINAL=2 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -x c "$<" -c -o "$@")
	@clang --target=arm-none-eabi @"$@.in"

