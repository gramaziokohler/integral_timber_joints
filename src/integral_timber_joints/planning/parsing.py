import os
import json
from copy import deepcopy
from termcolor import cprint

from compas.robots import RobotModel
from compas_fab.robots import RobotSemantics
from compas.utilities import DataDecoder, DataEncoder

from pybullet_planning import get_date
from integral_timber_joints.process import RoboticFreeMovement, RoboticLinearMovement, RoboticMovement, RobotClampAssemblyProcess

HERE = os.path.dirname(__file__)
EXTERNAL_DIR = os.path.abspath(os.path.join(HERE, '..', '..', '..', 'external'))
PLANNING_DATA_DIR = os.path.join(HERE, "data")
DESIGN_STUDY_DIR = os.path.abspath(os.path.join(EXTERNAL_DIR, 'itj_design_study'))

# TODO specific to RemodelFredPavilion now, change to parameter later
DESIGN_DIR = os.path.join(DESIGN_STUDY_DIR, '210128_RemodelFredPavilion')
TEMP_SUBDIR = 'YJ_tmp'

###########################################

def rfl_setup(model_dir=EXTERNAL_DIR):
    """construct RFL robot urdf path and a SRDF instance

    Parameters
    ----------
    model_dir : str, optional
        path to where the `rfl_description` is saved, by default EXTERNAL_DIR

    Returns
    -------
    (urdf file path, RobotSemantics)
    """
    # ! the original rfl.urdf use concave collision objects, over-conservative
    # urdf_filename = os.path.join(model_dir, 'rfl_description', 'rfl_description', "urdf", "rfl.urdf")
    urdf_filename = os.path.join(model_dir, 'rfl_description', 'rfl_description', "urdf", "rfl_pybullet.urdf")
    srdf_filename = os.path.join(model_dir, 'rfl_description', 'rfl_description', "urdf", "rfl.srdf")
    model = RobotModel.from_urdf_file(urdf_filename)
    semantics = RobotSemantics.from_srdf_file(srdf_filename, model)
    return urdf_filename, semantics

###########################################

def get_process_path(assembly_name, subdir='.'):
    if assembly_name.endswith('.json'):
        filename = os.path.basename(assembly_name)
    else:
        filename = '{}.json'.format(assembly_name)
    model_path = os.path.abspath(os.path.join(DESIGN_DIR, subdir, filename))
    return model_path

def parse_process(process_name, subdir='.') -> RobotClampAssemblyProcess:
    """parse a Process instance from a given process file name.

    Parameters
    ----------
    process_name : str
        Name of the process, e.g. 'twelve_pieces_process.json' or 'twelve_pieces_process'
    subdir : str, optional
        subdirectory of the design, e.g. '.' (for original) or 'YJ_tmp'.

    Returns
    -------
    Process

    Raises
    ------
    FileNotFoundError
        if file not found
    """
    # * Load process from file
    file_path = get_process_path(process_name, subdir)
    if not os.path.exists(file_path):
        cprint('No temp process file found, using the original one.', 'yellow')
        file_path = get_process_path(process_name, '.')
    if not os.path.exists(file_path):
        raise FileNotFoundError(file_path)

    with open(file_path, 'r') as f:
        process = json.load(f, cls=DataDecoder)
        # type: RobotClampAssemblyProcess
    cprint('Process json parsed from {}'.format(file_path), 'blue')
    return process

def save_process(_process, save_path, include_traj_in_process=False, indent=None):
    process = deepcopy(_process)
    if not include_traj_in_process:
        for m in process.movements:
            if isinstance(m, RoboticMovement):
                m.trajectory = None
    with open(save_path, 'w') as f:
        json.dump(process, f, cls=DataEncoder, indent=indent, sort_keys=True)

def save_process_and_movements(process_name, _process, _movements,
    overwrite=False, include_traj_in_process=False, indent=None, save_temp=True):
    """[summary]

    Parameters
    ----------
    process_name : str
        Name of the process, e.g. 'twelve_pieces_process.json' or 'twelve_pieces_process'
    _process : itj Process
        [description]
    _movements : list(tj Movement)
        [description]
    overwrite : bool, optional
        if set True, a new timestamped folder will be created and new process and movements will saved there, avoid overwriting
        existing files, by default False
    include_traj_in_process : bool, optional
        include the `trajectory` attribute in the movement or not, by default False
    indent : int, optional
        json indentation, by default None
    save_temp : bool, optional
        save an extra copy of the process in the `TEMP_DESIGN_DIR` folder, by default True
    """
    process = deepcopy(_process)
    movements = deepcopy(_movements)

    process_file_path = get_process_path(process_name, '.')
    process_dir = DESIGN_DIR
    if not overwrite:
        process_fname = os.path.basename(process_file_path)
        # time_stamp = get_date()
        save_dir = 'results'
        process_dir = os.path.join(process_dir, save_dir)
        # * make paths
        if not os.path.exists(process_dir):
            os.makedirs(process_dir)
        process_file_path = os.path.join(process_dir, process_fname)

    movement_dir = os.path.join(process_dir, 'movements')
    if not os.path.exists(movement_dir):
        os.makedirs(movement_dir)

    for m in movements:
        m_file_path = os.path.abspath(os.path.join(process_dir, m.filepath))
        with open(m_file_path, 'w') as f:
            json.dump(m, f, cls=DataEncoder, indent=indent, sort_keys=True)
    print('---')
    cprint('#{} movements written to {}'.format(len(movements), os.path.abspath(movement_dir)), 'green')

    if not include_traj_in_process:
        for m in process.movements:
            if isinstance(m, RoboticMovement):
                m.trajectory = None

    with open(process_file_path, 'w') as f:
        json.dump(process, f, cls=DataEncoder, indent=indent, sort_keys=True)
    print('---')
    cprint('Process written to {}'.format(process_file_path), 'green')

    if save_temp:
        temp_dir = os.path.join(DESIGN_DIR, TEMP_SUBDIR)
        if not os.path.exists(temp_dir):
            os.makedirs(temp_dir)
        temp_process_file_path = get_process_path(process_name, TEMP_SUBDIR)
        with open(temp_process_file_path, 'w') as f:
            json.dump(process, f, cls=DataEncoder, indent=indent, sort_keys=True)
        print('---')
        cprint('(extra copy) Process written to {}'.format(temp_process_file_path), 'green')


##########################################
