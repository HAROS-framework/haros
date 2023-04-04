# SPDX-License-Identifier: MIT
# Copyright © 2021 André Santos

"""
Module that contains the command line sub-program.
"""

###############################################################################
# Imports
###############################################################################

from typing import Any, Dict, Final, List

import argparse
import logging
from pathlib import Path

from haros.analysis.launch import get_launch_description
from haros.internal.fsutil import is_ros_package, is_workspace, StorageManager
from haros.internal.interface import AnalysisSystemInterface
from haros.internal.plugins import load as load_plugins
from haros.internal.settings import Settings
from haros.metamodel.builder.launch import model_from_description
from haros.metamodel.builder.projects import build_from_package_paths
from haros.metamodel.ros import ProjectModel

###############################################################################
# Constants
###############################################################################

DEFAULT_PROJECT: Final[str] = 'my-ros-project'

DEFAULT_FILE_NAME: Final[str] = 'project.haros.yaml'
DEFAULT_PATH: Final[Path] = Path.cwd() / DEFAULT_FILE_NAME

logger: Final[logging.Logger] = logging.getLogger(__name__)

###############################################################################
# Entry Point
###############################################################################


def subprogram(argv: List[str], settings: Settings) -> int:
    args = parse_arguments(argv)
    return run(args, settings)


###############################################################################
# Command-specific Functions
###############################################################################


def run(args: Dict[str, Any], settings: Settings) -> int:
    plugins = load_plugins(settings.home, settings.plugins)
    paths = args['paths']
    if args['packages']:
        logger.error('analysis: discovery mode not yet supported')
        return 1
    storage = process_paths(paths)
    if not storage.packages:
        logger.error('analysis: did not find any ROS packages')
        return 1
    logger.info(f'analysis: packages: {storage.packages}')
    plugins.on_analysis_begin()
    # print(f'analysis: packages: {list(storage.packages.keys())}')
    model = build_from_package_paths(args['name'], storage.packages)
    system = _setup_interface(storage, model)
    print('project:', model.name)
    for package in model.packages.values():
        print('  package:', package.name)
        for fp in package.files:
            file = model.files[f'{package.name}/{fp}']
            print('    file:', file.path, f'({file.source.language.value})')
            p = Path(file.path)
            if p.parts[0] == 'launch':
                print('      (within launch directory)')
                if '.launch' in p.suffixes:
                    print('      (maybe launch file)')
                    launch_description = get_launch_description(p)
                    m = model_from_description(p, launch_description, system)
                    print('Launch Model:')
                    print_launch_model(m)
    for node in model.nodes.values():
        print('  node:', node.uid, f'({node.source.language.value})')
        for uid in node.files:
            print('    file:', uid)
    plugins.on_analysis_end()
    return 0


###############################################################################
# Argument Parsing
###############################################################################


def parse_arguments(argv: List[str]) -> Dict[str, Any]:
    msg = 'Run analyses over ROS workspaces and packages'
    parser = argparse.ArgumentParser(prog='haros analysis', description=msg)

    parser.add_argument(
        'paths',
        nargs='*',
        default=[Path.cwd()],
        type=Path,
        help='paths to workspaces or packages [default: "."]',
    )

    parser.add_argument(
        '-p',
        '--packages',
        action='store_true',
        help=f'process args as package names',
    )

    parser.add_argument(
        '-n',
        '--name',
        default=DEFAULT_PROJECT,
        help=f'project name [default: {DEFAULT_PROJECT}]',
    )

    args = parser.parse_args(args=argv)
    return vars(args)


###############################################################################
# Argument Validation and Processing
###############################################################################


def process_paths(paths: List[Path]) -> StorageManager:
    storage = StorageManager()
    adhoc = []
    for path in paths:
        if is_ros_package(path):
            adhoc.append(path)
        else:
            storage.workspaces.append(path)
            if not is_workspace(path):
                logger.warning(f'analysis: workspace without "src" directory: {str(path)}')
    storage.crawl()
    for path in adhoc:
        storage.packages[path.name] = path
    return storage


###############################################################################
# Helper Functions
###############################################################################


def _setup_interface(storage: StorageManager, model: ProjectModel) -> AnalysisSystemInterface:
    workspace = Path.cwd() / 'tests' / 'ws1'  # FIXME
    repo = workspace / 'src' / 'repo'
    return AnalysisSystemInterface(
        workspace=str(workspace),
        packages={name: path.as_posix() for name, path in storage.packages.items()},
        model=model,
        parse_launch_description=get_launch_description,
    )
