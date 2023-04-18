# SPDX-License-Identifier: MIT
# Copyright © 2021 André Santos

"""
Module that contains the command line program.

Why does this file exist, and why not put this in __main__?

  In some cases, it is possible to import `__main__.py` twice.
  This approach avoids that. Also see:
  https://click.palletsprojects.com/en/5.x/setuptools/#setuptools-integration

Some of the structure of this file came from this StackExchange question:
  https://softwareengineering.stackexchange.com/q/418600
"""

###############################################################################
# Imports
###############################################################################

from typing import Any, Dict, List, Optional

import argparse
import json
import logging
from pathlib import Path
import time
import sys

from bottle import request, route, run, static_file
from harosviz import __version__ as current_version

###############################################################################
# Argument Parsing
###############################################################################


def parse_arguments(argv: Optional[List[str]]) -> Dict[str, Any]:
    msg = 'HAROSViz: Visualizer for ROS applications.'
    parser = argparse.ArgumentParser(description=msg)

    parser.add_argument(
        '--version',
        action='version',
        version=f'{current_version}',
        help='Prints the program version.'
    )

    parser.add_argument(
        '--root',
        metavar='DIR',
        # nargs=argparse.OPTIONAL,
        default='.',
        help='Root directory for the visualizer client. Defaults to the current directory.',
    )

    parser.add_argument(
        '--src',
        metavar='SRC',
        nargs=argparse.ZERO_OR_MORE,
        help='Directories containing ROS packages. Defaults to the current directory.',
    )

    args = parser.parse_args(args=argv)
    return vars(args)


###############################################################################
# Setup
###############################################################################


def load_configs(args: Dict[str, Any]) -> Dict[str, Any]:
    try:
        config: Dict[str, Any] = {}
        # with open(args['config_path'], 'r') as file_pointer:
        # yaml.safe_load(file_pointer)

        # arrange and check configs here

        return config
    except Exception as err:
        # log or raise errors
        print(err, file=sys.stderr)
        if str(err) == 'Really Bad':
            raise err

        # Optional: return some sane fallback defaults.
        sane_defaults: Dict[str, Any] = {}
        return sane_defaults


###############################################################################
# Commands
###############################################################################


def workflow(args: Dict[str, Any], configs: Dict[str, Any]) -> None:
    print(f'Arguments: {args}')
    print(f'Configurations: {configs}')

    root = args['root']
    path = Path(root).resolve()
    if not path.is_dir():
        raise ValueError(f'"{root}" is not a directory')

    global workspace
    workspace = fsys.Workspace(list(args['src']))
    workspace.find_packages()
    global ros_iface
    ros_iface = SimpleRosInterface(strict=True, pkgs=workspace.packages)
    ros_iface.executables = get_all_package_executables()

    set_routes(str(path), configs)
    run(host='localhost', port=8080)


###############################################################################
# Bottle
###############################################################################


def set_routes(root: str, configs: Dict[str, Any]):
    logger = configs['logger']

    def serve_file(filepath):
        return static_file(filepath, root=root)

    def get_feature_model(project_id=None):
        return _get_feature_model(root, project_id)

    def calculate_computation_graph():
        return _calculate_computation_graph(root)

    def query_computation_graph():
        return _query_computation_graph(root, logger)

    route('/')(lambda: serve_file('index.html'))
    route('/data/<project_id>/feature-model.json', method='GET')(get_feature_model)
    route('/data/<project_id>/feature-model.json', method='PUT')(_update_feature_model)
    route('/cg/calculate', method='POST')(calculate_computation_graph)
    route('/cg/query', method='POST')(query_computation_graph)
    route('/<filepath:path>')(serve_file)


###############################################################################
# Decorators
###############################################################################


def timed(fun):
    def wrapper(*args, **kwargs):
        t0 = time.time()
        res = fun(*args, **kwargs)
        delta = time.time() - t0
        if delta < 0:
            print(f'<done in {(delta * 1000):.2f} milliseconds>')
        else:
            print(f'<done in {delta:.2f} seconds>')
        return res
    return wrapper


###############################################################################
# Application Logic
###############################################################################

project_model: ProjectModel = None
project_nodes: List[Any] = []
current_cg: RosComputationGraph = None
query_engine = None
workspace = None
ros_iface = None


@timed
def _get_feature_model(root, project_id):
    print(f'Get: {project_id}/feature-model.json')
    _load_project_model(root, project_id)
    return _viz_feature_model_json(project_model)


@timed
def _update_feature_model(project_id=None):
    print(f'Update: {project_id}/feature-model.json')
    data = request.json
    print(f'JSON data:', data)
    if data['children']:
        data['children'][0]['name'] = 'modified'
    return data


@timed
def _calculate_computation_graph(root: str):
    global current_cg
    selection = request.json
    project_id = selection['project']
    print(f'Calculate Computation Graph for project "{project_id}"')
    _load_project_model(root, project_id)
    # current_cg = build_computation_graph_adhoc(project_model, data, project_nodes)
    current_cg = build_computation_graph_adhoc(
        project_model,
        selection,
        project_nodes,
        workspace,
        ros_iface=ros_iface,
    )
    cg = _cg_to_old_format(current_cg)
    _debug_no_solver_result(cg)
    return cg


@timed
def _query_computation_graph(root: str, logger):
    if current_cg is None:
        return {'error': 'Must calculate CG first.'}

    global query_engine
    if query_engine is None:
        try:
            from harosviz.query_engine import QueryEngine
        except ImportError as e:
            return {'error': f'Unable to import query engine: {e}'}
        temp_dir = Path(root).resolve() / 'data'
        query_engine = QueryEngine(str(temp_dir))

    data = request.json
    try:
        result, resources = query_engine.execute(data['query'], current_cg)
    except Exception as e:
        logger.exception(e)
        print(f'Error executing {data}: {e}')
        return {'error': str(e)}
    response = _old_query_results(current_cg, result, resources)
    logger.debug(f'Query response:\n{response}')
    return response


###############################################################################
# Helpers
###############################################################################


def _load_project_model(root, project_id):
    global project_model
    global project_nodes
    global current_cg
    if project_model is None or project_model.name != project_id:
        path = Path(root).resolve() / 'data' / project_id / 'model.json'
        if path.is_file():
            data = json.loads(path.read_text(encoding='utf-8'))
            project_model = ProjectModel.from_json(data)
        else:
            project_model = build_project(project_id, workspace)
            data = json.dumps(project_model.serialize(), ensure_ascii=False, indent=4)
            path.write_text(data, encoding='utf-8')
        project_nodes = _load_project_nodes(root)
        # selection = {
        #     'project': project_id,
        #     'launch': [],
        #     'discard': [],
        # }
        # nodes = _load_project_nodes(root)
        # current_cg = build_computation_graph_adhoc(project_model, selection, nodes)
        current_cg = None


def _load_project_nodes(root):
    path = Path(root).resolve() / 'data' / project_model.name / 'nodes.json'
    data = json.loads(path.read_text(encoding='utf-8'))
    nodes = []
    for item in data:
        package = item['package']
        nodes.append(Node(
            package,
            item['nodelet'] or item['name'],
            files=[FileId(f'{package}/{fp}') for fp in item['files']],
            advertise_calls=item['advertise'],
            subscribe_calls=item['subscribe'],
            srv_server_calls=item['service'],
            srv_client_calls=item['client'],
            param_get_calls=item['readParam'],
            param_set_calls=item['writeParam'],
        ))
    return nodes

###############################################################################
# Entry Point
###############################################################################


def main(argv: Optional[List[str]] = None) -> int:
    args = parse_arguments(argv)
    try:
        config = load_configs(args)
        logging.basicConfig(level=logging.DEBUG)
        logger = logging.getLogger(__name__)
        config['logger'] = logger
        workflow(args, config)
    except KeyboardInterrupt:
        logger.error('Aborted manually.')
        return 1
    except Exception as err:
        logger.exception(err)
        return 1
    return 0  # success
