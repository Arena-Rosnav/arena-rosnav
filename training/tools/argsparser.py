import argparse
import os
import numpy as np


def new_training_args(parser):
    """program arguments training script"""
    parser.add_argument(
        "--config",
        type=str,
        metavar="[config name]",
        default="training_config.yaml",
        help="name of the config file",
    )


def parse_training_args(args=None, ignore_unknown=True):
    """parser for training script"""
    arg_populate_funcs = [new_training_args]
    arg_check_funcs = []

    return parse_various_args(args, arg_populate_funcs, arg_check_funcs, ignore_unknown)


def parse_various_args(args, arg_populate_funcs, arg_check_funcs, ignore_unknown):
    """generic arg parsing function"""
    parser = argparse.ArgumentParser()

    for func in arg_populate_funcs:
        func(parser)

    if ignore_unknown:
        parsed_args, unknown_args = parser.parse_known_args(args=args)
    else:
        parsed_args = parser.parse_args(args=args)
        unknown_args = []

    for func in arg_check_funcs:
        func(parsed_args)

    print_args(parsed_args)
    return parsed_args, unknown_args


def print_args(args):
    print("\n-------------------------------")
    print("            ARGUMENTS          ")
    for k in args.__dict__:
        print("- {} : {}".format(k, args.__dict__[k]))
    print("--------------------------------\n")
