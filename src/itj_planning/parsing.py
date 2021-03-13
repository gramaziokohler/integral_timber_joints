import os
import json
import sys
from copy import deepcopy
from termcolor import cprint

from compas.datastructures import Mesh
from compas.robots import RobotModel
from compas_fab.robots import RobotSemantics
from compas_fab.robots import CollisionMesh
from compas.utilities import DataDecoder, DataEncoder

from pybullet_planning import get_date

from integral_timber_joints.process import RoboticFreeMovement, RoboticLinearMovement, RoboticMovement

HERE = os.path.dirname(__file__)
DATA_DIR = os.path.join(HERE, "data")

#######################################

# TODO made a GH script for auto-gen URDF for static collision objects
def parse_collision_mesh_from_path(dir_path, filename, scale=1e-3):
    file_path = os.path.join(dir_path, filename)
    obj_name = filename.split('.')[0]
    if filename.endswith('.obj'):
        mesh = Mesh.from_obj(file_path)
    elif filename.endswith('.stl'):
        mesh = Mesh.from_stl(file_path)
    else:
        return None
    cm = CollisionMesh(mesh, obj_name)
    cm.scale(scale)
    return cm

def parse_collision_meshes_from_dir(dir_path, scale=1e-3):
    cms = []
    for filename in sorted(os.listdir(dir_path)):
        cm = parse_collision_mesh_from_path(dir_path, filename, scale)
        if cm is not None:
            cms.append(cm)
    return cms

########################################
# ! DEPRECATED, these objects are parsed from Process json file
ARCHIVED_DATA_DIR = os.path.join(HERE, "data", "_archive")

def itj_rfl_obstacle_cms():
    dir_path = os.path.abspath(os.path.join(ARCHIVED_DATA_DIR, 'itj_rfl_obstacles'))
    return parse_collision_meshes_from_dir(dir_path)

###########################################

def rfl_setup():
    data_dir = os.path.abspath(os.path.join(HERE, "..", "..", "data", 'robots'))
    # ! the original rfl.urdf use concave collision objects, over-conservative
    # urdf_filename = os.path.join(data_dir, 'rfl_description', 'rfl_description', "urdf", "rfl.urdf")
    urdf_filename = os.path.join(data_dir, 'rfl_description', 'rfl_description', "urdf", "rfl_pybullet.urdf")
    srdf_filename = os.path.join(data_dir, 'rfl_description', 'rfl_description', "urdf", "rfl.srdf")
    model = RobotModel.from_urdf_file(urdf_filename)
    semantics = RobotSemantics.from_srdf_file(srdf_filename, model)
    return urdf_filename, semantics

###########################################

DESIGN_DIR = os.path.abspath(os.path.join(HERE, '..', '..', '..', 'itj_design_study', '210128_RemodelFredPavilion'))
TEMP_DESIGN_DIR = os.path.abspath(os.path.join(HERE, '..', '..', '..', 'itj_design_study', '210128_RemodelFredPavilion', 'YJ_tmp'))

def get_process_path(assembly_name, file_dir=DESIGN_DIR):
    if assembly_name.endswith('.json'):
        filename = os.path.basename(assembly_name)
    else:
        filename = '{}.json'.format(assembly_name)
    model_path = os.path.abspath(os.path.join(file_dir, filename))
    return model_path

def parse_process(process_name, parse_temp=False):
    # * Load process from file
    file_path = get_process_path(process_name, file_dir=TEMP_DESIGN_DIR if parse_temp else DESIGN_DIR)
    if parse_temp and not os.path.exists(file_path):
        cprint('No temp process file found, using the original one.', 'yellow')
        file_path = get_process_path(process_name, file_dir=DESIGN_DIR)
    if not os.path.exists(file_path):
        raise FileNotFoundError(file_path)

    with open(file_path, 'r') as f:
        process = json.load(f, cls=DataDecoder)
        # type: RobotClampAssemblyProcess
    cprint('Process json parsed from {}'.format(file_path), 'green')
    return process

def save_process_and_movements(process_name, _process, _movements, overwrite=False, include_traj_in_process=False, indent=None, save_temp=True):
    process = deepcopy(_process)
    movements = deepcopy(_movements)

    process_file_path = get_process_path(process_name, file_dir=DESIGN_DIR)
    process_dir = os.path.dirname(process_file_path)
    if not overwrite:
        process_fname = os.path.basename(process_file_path)
        time_stamp = get_date()
        process_dir = os.path.join(process_dir, time_stamp)
        # * make paths
        os.makedirs(process_dir)
        os.makedirs(os.path.join(process_dir, 'movements'))
        process_file_path = os.path.join(process_dir, process_fname)
        # process_fnames = os.path.basename(process_file_path).split('.json')
        # process_file_path = os.path.join(process_dir, process_fnames[0] + ('' if overwrite else '_'+get_date()) + '.json')

    for m in movements:
        m_file_path = os.path.abspath(os.path.join(process_dir, m.filepath))
        with open(m_file_path, 'w') as f:
            json.dump(m, f, cls=DataEncoder, indent=indent, sort_keys=True)
    print('---')
    cprint('#{} movements written to {}'.format(len(movements), os.path.abspath(DESIGN_DIR)), 'green')

    if not include_traj_in_process:
        for m in process.movements:
            if isinstance(m, RoboticMovement):
                m.trajectory = None

    with open(process_file_path, 'w') as f:
        json.dump(process, f, cls=DataEncoder, indent=indent, sort_keys=True)
    print('---')
    cprint('Process written to {}'.format(process_file_path), 'green')

    if save_temp:
        if not os.path.exists(TEMP_DESIGN_DIR):
            os.makedirs(TEMP_DESIGN_DIR)
        temp_process_file_path = get_process_path(process_name, file_dir=TEMP_DESIGN_DIR)
        with open(temp_process_file_path, 'w') as f:
            json.dump(process, f, cls=DataEncoder, indent=indent, sort_keys=True)
        print('---')
        cprint('(extra copy) Process written to {}'.format(temp_process_file_path), 'green')


##########################################
