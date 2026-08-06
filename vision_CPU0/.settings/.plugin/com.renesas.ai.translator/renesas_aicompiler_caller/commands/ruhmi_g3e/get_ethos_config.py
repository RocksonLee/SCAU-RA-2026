#######################################################################################################################
# DISCLAIMER
# This software is supplied by Renesas Electronics Corporation and is only intended for use with Renesas products. No
# other uses are authorized. This software is owned by Renesas Electronics Corporation and is protected under all
# applicable laws, including copyright laws.
# THIS SOFTWARE IS PROVIDED "AS IS" AND RENESAS MAKES NO WARRANTIES REGARDING
# THIS SOFTWARE, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING BUT NOT LIMITED TO WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT. ALL SUCH WARRANTIES ARE EXPRESSLY DISCLAIMED. TO THE MAXIMUM
# EXTENT PERMITTED NOT PROHIBITED BY LAW, NEITHER RENESAS ELECTRONICS CORPORATION NOR ANY OF ITS AFFILIATED COMPANIES
# SHALL BE LIABLE FOR ANY DIRECT, INDIRECT, SPECIAL, INCIDENTAL OR CONSEQUENTIAL DAMAGES FOR ANY REASON RELATED TO
# THIS SOFTWARE, EVEN IF RENESAS OR ITS AFFILIATES HAVE BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.
# Renesas reserves the right, without notice, to make changes to this software and to discontinue the availability of
# this software. By using this software, you agree to the additional terms and conditions found by accessing the
# following link:
# http://www.renesas.com/disclaimer
#
# Copyright (C) 2023-2024 Renesas Electronics Corporation. All rights reserved.
#######################################################################################################################
# commands/ruhmi_g3e/get_ethos_config.py

import sys
import argparse
from pathlib import Path
from renesas_aicompiler_caller.utils.environment_check import check_ruhmi_environment
from renesas_aicompiler_caller.utils import LogFactory

logger = LogFactory().add_console_handler() \
                     .build(__name__)

_G3E_ETHOS_U55_CONFIG = '''[architecture]
  macs=256

[System_Config.RA8P1]
core_clock=500e6
axi0_port=Sram
axi1_port=OffChipFlash

Sram_clock_scale=0.5
Sram_burst_length=128
Sram_read_latency=8
Sram_write_latency=8

OffChipFlash_clock_scale=0.5
OffChipFlash_burst_length=128
OffChipFlash_read_latency=8
OffChipFlash_write_latency=8

[Memory_Mode.Sram_Only]
const_mem_area=Axi0
arena_mem_area=Axi0
cache_mem_area=Axi0

[Memory_Mode.Shared_Sram]
const_mem_area=Axi1
arena_mem_area=Axi0
cache_mem_area=Axi0
'''


def generate_ethos_config(output_dir: str):
    """Generate config from embedded template

    Args:
        output_dir: Output directory for Ethos system configuration file

    Returns:
        Path to the generated config file
    """
    dst_path = Path(output_dir)
    dst_path.mkdir(parents=True, exist_ok=True)

    config_file = dst_path / "system_config.ini"

    # Write embedded config
    logger.info(f"Generating G3E Ethos config to: {config_file}")
    with open(config_file, 'w') as f:
        f.write(_G3E_ETHOS_U55_CONFIG)

    return config_file


def main(args):
    """Main function for command-line usage"""
    check_ruhmi_environment()
    parser = argparse.ArgumentParser(prog="ruhmi_g3e get_ethos_config")

    parser.add_argument(
        '--output_dir',
        required=True,
        type=str,
        help='Output directory for G3E Ethos system configuration file'
    )

    args = parser.parse_args(args)

    config_path = generate_ethos_config(args.output_dir)
    logger.info(f"SUCCESS: G3E Ethos configuration generated at {config_path}")


if __name__ == "__main__":
    main(sys.argv[1:])
