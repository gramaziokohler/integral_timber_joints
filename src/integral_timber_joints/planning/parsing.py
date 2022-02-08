import os
import shutil
import json
from copy import deepcopy
from compas_fab.backends.pybullet.utils import LOG
from termcolor import colored

from compas.robots import RobotModel
from compas_fab.robots import RobotSemantics
from compas.utilities import DataDecoder, DataEncoder

from pybullet_planning import get_date
from integral_timber_joints.process import RoboticFreeMovement, RoboticLinearMovement, RoboticMovement, RobotClampAssemblyProcess
from .utils import LOGGER

HERE = os.path.dirname(__file__)
EXTERNAL_DIR = os.path.abspath(os.path.join(HERE, '..', '..', '..', 'external'))
PLANNING_DATA_DIR = os.path.join(HERE, "data")
DESIGN_STUDY_DIR = os.path.abspath(os.path.join(EXTERNAL_DIR, 'itj_design_study'))

# DESIGN_DIR = os.path.join(DESIGN_STUDY_DIR, '210605_ScrewdriverTestProcess') # '210128_RemodelFredPavilion')
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

def mkdir(path):
    if not os.path.exists(path):
        os.makedirs(path)

###########################################

def get_process_path(design_dir, assembly_name, subdir='.'):
    if assembly_name.endswith('.json'):
        filename = os.path.basename(assembly_name)
    else:
        filename = '{}.json'.format(assembly_name)
    folder_dir = os.path.abspath(os.path.join(DESIGN_STUDY_DIR, design_dir, subdir))
    model_path = os.path.join(folder_dir, filename)
    if not os.path.exists(folder_dir):
        os.makedirs(folder_dir)
    return model_path

def parse_process(design_dir, process_name, subdir='.') -> RobotClampAssemblyProcess:
    """parse a Process instance from a given process file name.

    Parameters
    ----------
    design_dir : str
        Design's name (aka name of the process json's containing folder), e.g. '210605_ScrewdriverTestProcess'
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
    file_path = get_process_path(design_dir, process_name, subdir)
    if not os.path.exists(file_path):
        LOGGER.warning('No process file found, using the original one.', 'yellow')
        file_path = get_process_path(design_dir, process_name, '.')
    if not os.path.exists(file_path):
        raise FileNotFoundError(file_path)

    with open(file_path, 'r') as f:
        process = json.load(f, cls=DataDecoder)
        # type: RobotClampAssemblyProcess
    LOGGER.debug(colored('Process json parsed from {}'.format(file_path), 'blue'))

    # * Double check entire solution is valid
    for beam_id in process.assembly.sequence:
        if not process.dependency.beam_all_valid(beam_id):
            process.dependency.compute_all(beam_id)
            assert process.dependency.beam_all_valid(beam_id)
    return process

def save_process(_process, save_path, include_traj_in_process=False, indent=None):
    process = deepcopy(_process)
    if not include_traj_in_process:
        for m in process.movements:
            if isinstance(m, RoboticMovement):
                m.trajectory = None
    with open(save_path, 'w') as f:
        json.dump(process, f, cls=DataEncoder, indent=indent, sort_keys=True)

def save_process_and_movements(design_dir, process_name, _process, _movements,
    overwrite=False, include_traj_in_process=False, indent=None, movement_subdir='movements'):
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
        if set False, a folder called `results` will be created and new process and movements will saved there, avoid overwriting
        existing files, by default False
    include_traj_in_process : bool, optional
        include the `trajectory` attribute in the movement or not, by default False
    indent : int, optional
        json indentation, by default None
    movement_subdir : str
        subdirectory name of the folder to save the movements, defaults to `movements`. Popular use: `smoothed_movements`
    """
    if not _movements:
        return

    process_file_path = get_process_path(design_dir, process_name, '.')
    process_fname = os.path.basename(process_file_path)

    process_dir = os.path.join(DESIGN_STUDY_DIR, design_dir)
    if not overwrite:
        # time_stamp = get_date()
        save_dir = 'results'
        process_dir = os.path.join(process_dir, save_dir)
        # * make paths
        if not os.path.exists(process_dir):
            os.makedirs(process_dir)

    process_file_path = os.path.join(process_dir, process_fname)

    movement_dir = os.path.join(process_dir, movement_subdir)
    if not os.path.exists(movement_dir):
        os.makedirs(movement_dir)

    for m in _movements:
        m_file_path = os.path.abspath(os.path.join(process_dir, m.get_filepath(movement_subdir)))
        with open(m_file_path, 'w') as f:
            json.dump(m, f, cls=DataEncoder, indent=indent, sort_keys=True)
    LOGGER.debug(colored('#{} movements written to {}'.format(len(_movements), os.path.abspath(movement_dir)), 'green'))

    process = deepcopy(_process)
    if not include_traj_in_process:
        for m in process.movements:
            if isinstance(m, RoboticMovement):
                m.trajectory = None
    with open(process_file_path, 'w') as f:
        json.dump(process, f, cls=DataEncoder, indent=indent, sort_keys=True)
    LOGGER.debug(colored('Process written to {}'.format(process_file_path), 'green'))

##########################################

def archive_saved_movement(movement, process_folder_path):
    smoothed_movement_path = os.path.join(process_folder_path, movement.get_filepath(subdir='smoothed_movements'))
    nonsmoothed_movement_path = os.path.join(process_folder_path, movement.get_filepath(subdir='movements'))
    archive_path = os.path.join(process_folder_path, 'archived')
    archive_smoothed_path = os.path.join(archive_path, 'smoothed_movements')
    archive_nonsmoothed_path = os.path.join(archive_path, 'movements')
    mkdir(archive_smoothed_path)
    mkdir(archive_nonsmoothed_path)
    movement_removed = False
    if os.path.exists(smoothed_movement_path):
        shutil.move(smoothed_movement_path, archive_smoothed_path)
        # LOGGER.info(f'{smoothed_movement_path} moved to {archive_smoothed_path}')
        movement_removed = True
    if os.path.exists(nonsmoothed_movement_path):
        shutil.move(nonsmoothed_movement_path, archive_nonsmoothed_path)
        # LOGGER.info(f'{nonsmoothed_movement_path} moved to {archive_nonsmoothed_path}')
        movement_removed = True
    return movement_removed

def archive_saved_movements(process, process_folder_path, beam_ids, movement_id=None):
    if movement_id is not None:
        if movement_id.startswith('A'):
            movement = process.get_movement_by_movement_id(movement_id)
        else:
            movement = process.movements[int(movement_id)]
        # only remove one movement
        archive_saved_movement(movement, process_folder_path)
    else:
        for beam_id in beam_ids:
            movements = process.get_movements_by_beam_id(beam_id)
            for m in movements:
                archive_saved_movement(m, process_folder_path)
