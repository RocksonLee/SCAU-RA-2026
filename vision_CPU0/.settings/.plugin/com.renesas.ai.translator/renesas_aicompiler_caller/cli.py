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
import sys
import os
import argparse
from pathlib import Path
import platform
import runpy

# To append the drpai tvm plug path
sys.path.append(str(Path(__file__).resolve().parent.parent))
import renesas_aicompiler_caller  # noqa

# Define command registry with module names instead of direct imports
registered_commands = None

# Define common commands available on both Windows and Linux platforms
common_commands = {
    'ruhmi deploy': (
        "Deploy a model with RUHMI",
        'commands.ruhmi.deploy_model'
    ),
    'ruhmi quantize': (
        "Quantize a model with RUHMI",
        'commands.ruhmi.quantize_model'
    ),
    'ruhmi get_ethos_config': (
        "Get a ethos system config from RUHMI",
        'commands.ruhmi.get_ethos_config'
    ),
    'ruhmi_g3e deploy': (
        "Deploy a model with RUHMI for G3E (INT8 TFLite only)",
        'commands.ruhmi_g3e.deploy_model'
    ),
    'ruhmi_g3e get_ethos_config': (
        "Get a ethos system config for G3E",
        'commands.ruhmi_g3e.get_ethos_config'
    ),
    'comm get_model_info': (
        "Get a model information",
        'commands.common.get_model_info'
    )
}

# Define Linux-specific commands that are not available on Windows
linux_only_commands = {
    'tvm gen_drpai_tvm_obj': (
        "Generate a DRP-AI TVM Model Object",
        'commands.tvm.gen_drpai_tvm_obj'
    ),
    'tvm gen_preprocess_obj': (
        "Generate a DRP-AI Preprocess Runtime Object",
        'commands.tvm.gen_preprocess_obj'
    ),
    'tvm gen_drpai_tvm_sample_app': (
        "Generate a sample code for DRP-AI TVM",
        'commands.tvm.gen_drpai_tvm_sample_app'
    ),
    'tvm compare_fp32_and_int8': (
        "Compare outputs between fp32 and int8 models",
        'commands.tvm.compare_fp32_and_int8'
    ),
    'comm convert_pth2pt': (
        "Convert the model format from pth to pt",
        'commands.common.pth2pt'
    )
}

# Determine the current platform and assign the appropriate command set
if platform.system() == "Windows":
    # On Windows, only common commands are available
    registered_commands = common_commands
elif platform.system() == "Linux":
    # On Linux, both common and Linux-specific commands are available
    registered_commands = {**common_commands, **linux_only_commands}
else:
    # Raise an error for unsupported platforms
    raise RuntimeError(f"Unsupported system was detected: {platform.system()}")


def build_parser():
    desc = "Available commands:\n"
    for cmd, (desc_text, _) in registered_commands.items():
        desc += f"  {cmd:<18} {desc_text}\n"

    parser = argparse.ArgumentParser(
        prog="cli.py",
        description=desc,
        formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument(
        "--device-type",
        required=False,
        choices=["RA8P1", "RA8", "RZG3E"],
        help="Target device type (e.g., RA8P1, RA8, RZG3E)"
    )
    parser.add_argument("top_command", nargs="?", help="Main command (e.g. ruhmi)")
    parser.add_argument("sub_command", nargs="?", help="Subcommand (e.g. deploy, quantize, get_ethos_config)")
    parser.add_argument("args", nargs=argparse.REMAINDER, help="Arguments for subcommand")
    return parser


def dispatch(argv):

    parser = build_parser()
    args = parser.parse_args(argv)

    if not args.top_command or not args.sub_command:
        parser.print_help()
        return

    command_key = f"{args.top_command} {args.sub_command}"

    if command_key not in registered_commands:
        print(f"Unknown command: {command_key}")
        parser.print_help()
        sys.exit(1)

    # Inject device type into environment variable
    if args.device_type:
        os.environ["RUHMI_DEVICE_TYPE"] = args.device_type

    _, module_name = registered_commands[command_key]

    # Pass arguments to the module via sys.argv
    original_argv = sys.argv.copy()
    sys.argv = [module_name] + args.args
    runpy.run_module(module_name, run_name="__main__")
    sys.argv = original_argv


if __name__ == '__main__':
    dispatch(sys.argv[1:])
