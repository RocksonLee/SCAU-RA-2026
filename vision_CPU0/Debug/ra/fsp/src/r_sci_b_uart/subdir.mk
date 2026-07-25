################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../ra/fsp/src/r_sci_b_uart/r_sci_b_uart.c 

C_DEPS += \
./ra/fsp/src/r_sci_b_uart/r_sci_b_uart.d 

CREF += \
vision_CPU0.cref 

OBJS += \
./ra/fsp/src/r_sci_b_uart/r_sci_b_uart.o 

MAP += \
vision_CPU0.map 


# Each subdirectory must supply rules for building sources it contributes
ra/fsp/src/r_sci_b_uart/%.o: ../ra/fsp/src/r_sci_b_uart/%.c
	@echo 'Building file: $<'
	$(file > $@.in,-mcpu=cortex-m85 -mthumb -mlittle-endian -mfloat-abi=hard -Os -ffunction-sections -fdata-sections -fno-strict-aliasing -fmessage-length=0 -funsigned-char -Wunused -Wuninitialized -Wall -Wextra -Wmissing-declarations -Wconversion -Wpointer-arith -Wshadow -Waggregate-return -Wno-parentheses-equality -Wfloat-equal -g3 -std=c99 -flax-vector-conversions -fshort-enums -fno-unroll-loops -I"D:\\e2studioworkspace\\SCAU-RA-2026\\vision_CPU0\\ra_cfg\\fsp_cfg\\bsp" -I"." -I"D:\\e2studioworkspace\\SCAU-RA-2026\\vision_CPU0\\ra_gen" -I"D:\\e2studioworkspace\\SCAU-RA-2026\\vision_CPU0\\ra_cfg\\fsp_cfg" -I"D:\\e2studioworkspace\\SCAU-RA-2026\\vision_CPU0\\ra_cfg\\aws" -I"D:\\e2studioworkspace\\SCAU-RA-2026\\vision_CPU0\\ra\\arm\\CMSIS_6\\CMSIS\\Core\\Include" -I"D:\\e2studioworkspace\\SCAU-RA-2026\\vision_CPU0\\src" -I"D:\\e2studioworkspace\\SCAU-RA-2026\\vision_CPU0\\src\\models" -I"D:\\e2studioworkspace\\SCAU-RA-2026\\vision_CPU0\\src\\hardware" -I"D:\\e2studioworkspace\\SCAU-RA-2026\\vision_CPU0\\ra\\fsp\\inc" -I"D:\\e2studioworkspace\\SCAU-RA-2026\\vision_CPU0\\ra\\fsp\\inc\\api" -I"D:\\e2studioworkspace\\SCAU-RA-2026\\vision_CPU0\\ra\\fsp\\inc\\instances" -I"D:\\e2studioworkspace\\SCAU-RA-2026\\vision_CPU0\\ra\\fsp\\src\\rm_freertos_port" -I"D:\\e2studioworkspace\\SCAU-RA-2026\\vision_CPU0\\ra\\aws\\FreeRTOS\\FreeRTOS\\Source\\include" -I"D:\\e2studioworkspace\\SCAU-RA-2026\\vision_CPU0\\ra\\npu\\ethos-u-core-driver\\include" -I"D:\\e2studioworkspace\\SCAU-RA-2026\\vision_CPU0\\ra\\arm\\CMSIS-NN\\Include" -I"D:\\e2studioworkspace\\SCAU-RA-2026\\vision_CPU0\\ra\\arm\\CMSIS-NN" -I"D:\\e2studioworkspace\\SCAU-RA-2026\\vision_CPU0\\ra\\npu\\ethos-u-core-software\\lib\\layer_by_layer_profiler\\include" -I"D:\\e2studioworkspace\\SCAU-RA-2026\\vision_CPU0\\ra\\npu\\ethos-u-core-software\\lib\\ethosu_monitor\\include" -I"D:\\e2studioworkspace\\SCAU-RA-2026\\vision_CPU0\\ra\\npu\\ethos-u-core-software\\lib\\ethosu_profiler\\include" -I"D:\\e2studioworkspace\\SCAU-RA-2026\\vision_CPU0\\ra\\npu\\ethos-u-core-software\\lib\\crc\\include" -I"D:\\e2studioworkspace\\SCAU-RA-2026\\vision_CPU0\\ra\\npu\\ethos-u-core-software\\lib\\arm_profiler\\include" -I"D:\\e2studioworkspace\\SCAU-RA-2026\\vision_CPU0\\ra\\arm\\CMSIS-View\\EventRecorder\\Include" -I"D:\\e2studioworkspace\\SCAU-RA-2026\\vision_CPU0\\ra\\arm\\CMSIS-View\\EventRecorder\\Config" -I"D:\\e2studioworkspace\\SCAU-RA-2026\\vision_CPU0\\ra\\npu\\tflite-micro" -I"D:\\e2studioworkspace\\SCAU-RA-2026\\vision_CPU0\\ra\\npu\\ruy" -I"D:\\e2studioworkspace\\SCAU-RA-2026\\vision_CPU0\\ra\\npu\\gemmlowp" -I"D:\\e2studioworkspace\\SCAU-RA-2026\\vision_CPU0\\ra\\fsp\\src\\rm_ethosu" -I"D:\\e2studioworkspace\\SCAU-RA-2026\\vision_CPU0\\ra\\arm\\CMSIS-DSP\\PrivateInclude" -I"D:\\e2studioworkspace\\SCAU-RA-2026\\vision_CPU0\\ra\\arm\\CMSIS-DSP\\Include" -I"D:\\e2studioworkspace\\SCAU-RA-2026\\vision_CPU0\\ra\\npu\\flatbuffers\\include" -D_RENESAS_RA_ -D_RA_CORE=CPU0 -D_RA_ORDINAL=1 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -x c "$<" -c -o "$@")
	@clang --target=arm-none-eabi @"$@.in"

