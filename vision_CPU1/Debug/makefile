################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

-include makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include src/hardware/subdir.mk
-include src/subdir.mk
-include ra_gen/subdir.mk
-include ra/fsp/src/rm_freertos_port/subdir.mk
-include ra/fsp/src/r_sci_b_uart/subdir.mk
-include ra/fsp/src/r_ioport/subdir.mk
-include ra/fsp/src/r_canfd/subdir.mk
-include ra/fsp/src/bsp/mcu/ra8p1/subdir.mk
-include ra/fsp/src/bsp/mcu/all/subdir.mk
-include ra/fsp/src/bsp/cmsis/Device/RENESAS/Source/subdir.mk
-include ra/board/ra8p1_ek/subdir.mk
-include ra/aws/FreeRTOS/FreeRTOS/Source/portable/MemMang/subdir.mk
-include ra/aws/FreeRTOS/FreeRTOS/Source/subdir.mk
-include ra/arm/CMSIS_6/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(JMP_DEPS)),)
-include $(JMP_DEPS)
endif
ifneq ($(strip $(FSY_DEPS)),)
-include $(FSY_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
ifneq ($(strip $(SRC_UPPER_DEPS)),)
-include $(SRC_UPPER_DEPS)
endif
ifneq ($(strip $(P_UPPER_DEPS)),)
-include $(P_UPPER_DEPS)
endif
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(PP_UPPER_DEPS)),)
-include $(PP_UPPER_DEPS)
endif
ifneq ($(strip $(P_DEPS)),)
-include $(P_DEPS)
endif
ifneq ($(strip $(FSY_UPPER_DEPS)),)
-include $(FSY_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
$(C_DEPS):$(SMS_ASM)
endif
ifneq ($(strip $(SRC_DEPS)),)
-include $(SRC_DEPS)
endif
ifneq ($(strip $(JMP_UPPER_DEPS)),)
-include $(JMP_UPPER_DEPS)
endif
ifneq ($(strip $(PP_DEPS)),)
-include $(PP_DEPS)
endif
ifneq ($(strip $(SX_DEPS)),)
-include $(SX_DEPS)
endif
ifneq ($(strip $(ASM_UPPER_DEPS)),)
-include $(ASM_UPPER_DEPS)
endif
ifneq ($(strip $(SX_UPPER_DEPS)),)
-include $(SX_UPPER_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
OBJCOPY += \
vision_CPU1.srec \

SECONDARY_SIZE += \
vision_CPU1.siz \


# All Target
all: vision_CPU1.elf secondary-outputs

# Tool invocations
vision_CPU1.elf: $(OBJS) $(USER_OBJS) $(LIBRARY_GENERATOR_OUTPUTTYPE_OUTPUTS) $(LINKER_SCRIPT)
	$(file > vision_CPU1.elf.in,-mcpu=cortex-m33 -mthumb -mlittle-endian -mfloat-abi=hard -mfpu=fpv5-sp-d16 -Os -ffunction-sections -fdata-sections -fno-strict-aliasing -fmessage-length=0 -funsigned-char -Wunused -Wuninitialized -Wall -Wextra -Wmissing-declarations -Wconversion -Wpointer-arith -Wshadow -Waggregate-return -Wno-parentheses-equality -Wfloat-equal -g3 -o "vision_CPU1.elf" $(OBJS) $(USER_OBJS) -T "fsp.lld" -L"D:\\RA_R8P_Workspace_copy\\vision_CPU1/script" -L"D:\\RA_R8P_Workspace_copy\\vision_CPU1\\Debug" -Wl,-Map=vision_CPU1.map -Wl,--start-group -lcrt0 -lc++abi  -Wl,--end-group -Wl,--gc-sections -Wl,--cref -Wl,-z,norelro -Wl,--icf=none -I"D:\\RA_R8P_Workspace_copy\\vision_CPU1\\ra_cfg\\fsp_cfg\\bsp" -I"." -I"D:\\RA_R8P_Workspace_copy\\vision_CPU1\\ra_gen" -I"D:\\RA_R8P_Workspace_copy\\vision_CPU1\\ra_cfg\\fsp_cfg" -I"D:\\RA_R8P_Workspace_copy\\vision_CPU1\\ra_cfg\\aws" -I"D:\\RA_R8P_Workspace_copy\\vision_CPU1\\ra\\arm\\CMSIS_6\\CMSIS\\Core\\Include" -I"D:\\RA_R8P_Workspace_copy\\vision_CPU1\\src" -I"D:\\RA_R8P_Workspace_copy\\vision_CPU1\\ra\\fsp\\inc" -I"D:\\RA_R8P_Workspace_copy\\vision_CPU1\\ra\\fsp\\inc\\api" -I"D:\\RA_R8P_Workspace_copy\\vision_CPU1\\ra\\fsp\\inc\\instances" -I"D:\\RA_R8P_Workspace_copy\\vision_CPU1\\ra\\fsp\\src\\rm_freertos_port" -I"D:\\RA_R8P_Workspace_copy\\vision_CPU1\\ra\\aws\\FreeRTOS\\FreeRTOS\\Source\\include")
	@echo Building target: $@ && clang  --target=arm-none-eabi @"vision_CPU1.elf.in"> vision_CPU1.cref
vision_CPU1.srec: vision_CPU1.elf
	llvm-objcopy "vision_CPU1.elf" -O srec  "vision_CPU1.srec"
vision_CPU1.siz: vision_CPU1.elf
	llvm-size --format=berkeley "vision_CPU1.elf"
# Other Targets
clean:
	-$(file > clean_file.tmp,  $(JMP_DEPS)  $(FSY_DEPS)  $(C_UPPER_DEPS)  $(SECONDARY_SIZE)  $(SRC_UPPER_DEPS)  $(P_UPPER_DEPS)  $(S_DEPS)  $(PP_UPPER_DEPS)  $(P_DEPS)  $(FSY_UPPER_DEPS)  $(C_DEPS)  $(CREF)  $(SRC_DEPS)  $(OBJCOPY)  $(JMP_UPPER_DEPS)  $(PP_DEPS)  $(SX_DEPS)  $(ASM_UPPER_DEPS)  $(OBJS)  $(SX_UPPER_DEPS)  $(ASM_DEPS)  $(MAP)  $(S_UPPER_DEPS) )
	-@ xargs -P 2 -s 32000 -t -a clean_file.tmp $(RM)
	-@$(RM) clean_file.tmp
	-$(RM)  vision_CPU1.elf
	-@echo ' '

secondary-outputs: $(OBJCOPY) $(SECONDARY_SIZE)

.PHONY: all clean dependents 

-include ../makefile.targets
