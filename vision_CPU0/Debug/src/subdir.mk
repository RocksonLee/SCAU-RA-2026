################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/Camera_thread_entry.c \
../src/Screen_Thread_entry.c \
../src/hal_warmstart.c 

C_DEPS += \
./src/Camera_thread_entry.d \
./src/Screen_Thread_entry.d \
./src/hal_warmstart.d 

CREF += \
vision_CPU0.cref 

OBJS += \
./src/Camera_thread_entry.o \
./src/Screen_Thread_entry.o \
./src/hal_warmstart.o 

MAP += \
vision_CPU0.map 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.c
	@echo 'Building file: $<'
	$(file > $@.in,-mcpu=cortex-m85 -mthumb -mlittle-endian -mfloat-abi=hard -Os -ffunction-sections -fdata-sections -fno-strict-aliasing -fmessage-length=0 -funsigned-char -Wunused -Wuninitialized -Wall -Wextra -Wmissing-declarations -Wconversion -Wpointer-arith -Wshadow -Waggregate-return -Wno-parentheses-equality -Wfloat-equal -g3 -std=c99 -flax-vector-conversions -fshort-enums -fno-unroll-loops -I"E:\\RA project\\SCAU-RA-2026\\vision_CPU0\\ra_cfg\\fsp_cfg\\bsp" -I"E:\\RA project\\SCAU-RA-2026\\vision_CPU0\\ra\\lvgl\\lvgl" -I"." -I"E:\\RA project\\SCAU-RA-2026\\vision_CPU0\\ra_gen" -I"E:\\RA project\\SCAU-RA-2026\\vision_CPU0\\ra_cfg\\fsp_cfg" -I"E:\\RA project\\SCAU-RA-2026\\vision_CPU0\\ra_cfg\\aws" -I"E:\\RA project\\SCAU-RA-2026\\vision_CPU0\\ra\\arm\\CMSIS_6\\CMSIS\\Core\\Include" -I"E:\\RA project\\SCAU-RA-2026\\vision_CPU0\\src" -I"E:\\RA project\\SCAU-RA-2026\\vision_CPU0\\src\\models" -I"E:\\RA project\\SCAU-RA-2026\\vision_CPU0\\src\\hardware" -I"E:\\RA project\\SCAU-RA-2026\\vision_CPU0\\ra\\fsp\\inc" -I"E:\\RA project\\SCAU-RA-2026\\vision_CPU0\\ra\\fsp\\inc\\api" -I"E:\\RA project\\SCAU-RA-2026\\vision_CPU0\\ra\\fsp\\inc\\instances" -I"E:\\RA project\\SCAU-RA-2026\\vision_CPU0\\ra\\fsp\\src\\rm_freertos_port" -I"E:\\RA project\\SCAU-RA-2026\\vision_CPU0\\ra\\aws\\FreeRTOS\\FreeRTOS\\Source\\include" -I"E:\\RA project\\SCAU-RA-2026\\vision_CPU0\\ra\\npu\\ethos-u-core-driver\\include" -I"E:\\RA project\\SCAU-RA-2026\\vision_CPU0\\ra\\arm\\CMSIS-NN\\Include" -I"E:\\RA project\\SCAU-RA-2026\\vision_CPU0\\ra\\arm\\CMSIS-NN" -I"E:\\RA project\\SCAU-RA-2026\\vision_CPU0\\ra\\npu\\ethos-u-core-software\\lib\\layer_by_layer_profiler\\include" -I"E:\\RA project\\SCAU-RA-2026\\vision_CPU0\\ra\\npu\\ethos-u-core-software\\lib\\ethosu_monitor\\include" -I"E:\\RA project\\SCAU-RA-2026\\vision_CPU0\\ra\\npu\\ethos-u-core-software\\lib\\ethosu_profiler\\include" -I"E:\\RA project\\SCAU-RA-2026\\vision_CPU0\\ra\\npu\\ethos-u-core-software\\lib\\crc\\include" -I"E:\\RA project\\SCAU-RA-2026\\vision_CPU0\\ra\\npu\\ethos-u-core-software\\lib\\arm_profiler\\include" -I"E:\\RA project\\SCAU-RA-2026\\vision_CPU0\\ra\\arm\\CMSIS-View\\EventRecorder\\Include" -I"E:\\RA project\\SCAU-RA-2026\\vision_CPU0\\ra\\arm\\CMSIS-View\\EventRecorder\\Config" -I"E:\\RA project\\SCAU-RA-2026\\vision_CPU0\\ra\\npu\\tflite-micro" -I"E:\\RA project\\SCAU-RA-2026\\vision_CPU0\\ra\\npu\\ruy" -I"E:\\RA project\\SCAU-RA-2026\\vision_CPU0\\ra\\npu\\gemmlowp" -I"E:\\RA project\\SCAU-RA-2026\\vision_CPU0\\ra\\fsp\\src\\rm_ethosu" -I"E:\\RA project\\SCAU-RA-2026\\vision_CPU0\\ra\\arm\\CMSIS-DSP\\PrivateInclude" -I"E:\\RA project\\SCAU-RA-2026\\vision_CPU0\\ra\\arm\\CMSIS-DSP\\Include" -I"E:\\RA project\\SCAU-RA-2026\\vision_CPU0\\ra\\npu\\flatbuffers\\include" -D_RENESAS_RA_ -D_RA_CORE=CPU0 -D_RA_ORDINAL=1 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -x c "$<" -c -o "$@")
	@clang --target=arm-none-eabi @"$@.in"

