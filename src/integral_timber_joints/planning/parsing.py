import os
import shutil
import json
from copy import deepcopy, copy
from termcolor import colored

from compas.robots import RobotModel
from compas_fab.robots import RobotSemantics
from compas.utilities import DataDecoder, DataEncoder

from integral_timber_joints.process import RoboticFreeMovement, RoboticLinearMovement, RoboticMovement, RobotClampAssemblyProcess
from .utils import LOGGER, robotic_movement_ids_from_beam_ids

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

def safe_rm_dir(d):
    if os.path.exists(d):
        shutil.rmtree(d)

###########################################

def get_process_path(design_dir, assembly_name, subdir='.'):
    if assembly_name.endswith('.json'):
        filename = os.path.basename(assembly_name)
    else:
        filename = '{}.json'.format(assembly_name)
    folder_dir = os.path.abspath(os.path.join(DESIGN_STUDY_DIR, design_dir, subdir))
    model_path = os.path.join(folder_dir, filename)
    mkdir(folder_dir)
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
    if os.path.exists(file_path):
        LOGGER.warning('Process file loaded from sub-directory: {}'.format(file_path))
    else:
        file_path = get_process_path(design_dir, process_name, '.')
    if os.path.exists(file_path):
        LOGGER.info('Process file loaded from main problem folder: {}'.format(file_path))
    else:
        LOGGER.error('Process file cannot be found in folder: {}'.format(file_path))
        raise FileNotFoundError(file_path)
    with open(file_path, 'r') as f:
        process = json.load(f, cls=DataDecoder)
    LOGGER.debug(colored('Process json parsed from {}'.format(file_path), 'blue'))
    # * Double check entire solution is valid
    for beam_id in process.assembly.sequence:
        if not process.dependency.beam_all_valid(beam_id):
            process.dependency.compute_all(beam_id)
            assert process.dependency.beam_all_valid(beam_id)
    return process

def save_process(design_dir, process_name, _process, save_dir='results', include_traj_in_process=False, indent=None):
    process_file_path = get_process_path(design_dir, process_name, save_dir)
    process = deepcopy(_process)
    if not include_traj_in_process:
        for m in process.movements:
            if isinstance(m, RoboticMovement):
                process.set_movement_trajectory(m, None)
    with open(process_file_path, 'w') as f:
        json.dump(process, f, cls=DataEncoder, indent=indent, sort_keys=True)
    LOGGER.debug(colored('Process written to {}'.format(process_file_path), 'green'))


def save_movements(design_dir, movements, save_dir='results', indent=None, movement_subdir='movements'):
    if len(movements) == 0:
        LOGGER.warning('No movement to be saved!')
        return
    process_dir = os.path.join(DESIGN_STUDY_DIR, design_dir, save_dir)
    movement_dir = os.path.join(process_dir, movement_subdir)
    mkdir(movement_dir)
    for m in movements:
        if isinstance(m, RoboticMovement) and m.trajectory is not None:
            # * skip saving non-robotic movement
            m_file_path = os.path.abspath(os.path.join(process_dir, m.get_filepath(movement_subdir)))
            with open(m_file_path, 'w') as f:
                json.dump(m, f, cls=DataEncoder, indent=indent, sort_keys=True)
    LOGGER.debug(colored('#{} movements written to {}'.format(len(movements), os.path.abspath(movement_dir)), 'green'))

##########################################

def move_saved_movement(movement, process_folder_path, to_archive=True):
    archive_path = os.path.join(process_folder_path, 'archived')
    mkdir(os.path.join(archive_path,'smoothed_movements'))
    mkdir(os.path.join(archive_path,'movements'))

    smoothed_path = os.path.join(process_folder_path, movement.get_filepath(subdir='smoothed_movements'))
    nonsmoothed_path = os.path.join(process_folder_path, movement.get_filepath(subdir='movements'))
    archive_smoothed_path = os.path.join(archive_path, movement.get_filepath(subdir='smoothed_movements'))
    archive_nonsmoothed_path = os.path.join(archive_path, movement.get_filepath(subdir='movements'))

    if to_archive:
        from_smoothed_path = smoothed_path
        from_nonsmoothed_path = nonsmoothed_path
        to_smoothed_path = archive_smoothed_path
        to_nonsmoothed_path = archive_nonsmoothed_path
    else:
        to_smoothed_path = smoothed_path
        to_nonsmoothed_path = nonsmoothed_path
        from_smoothed_path = archive_smoothed_path
        from_nonsmoothed_path = archive_nonsmoothed_path

    movement_removed = False
    if os.path.exists(from_smoothed_path):
        shutil.move(from_smoothed_path, to_smoothed_path)
        movement_removed = True
    if os.path.exists(from_nonsmoothed_path):
        shutil.move(from_nonsmoothed_path, to_nonsmoothed_path)
        movement_removed = True
    return movement_removed

def archive_robotic_movements(target_process, beam_ids, process_folder_path, movement_id=None):
    robotic_movement_ids = robotic_movement_ids_from_beam_ids(target_process, beam_ids, movement_id)
    for m_id in robotic_movement_ids:
        target_m = target_process.get_movement_by_movement_id(m_id)
        move_saved_movement(target_m, process_folder_path)

def copy_robotic_movements(source_process: RobotClampAssemblyProcess, target_process: RobotClampAssemblyProcess,
        beam_ids, movement_id=None, options=None):
    options = options or {}
    use_stored_seed = options.get('use_stored_seed', False)
    robotic_movement_ids = robotic_movement_ids_from_beam_ids(target_process, beam_ids, movement_id)
    saved_seed = None
    for m_id in robotic_movement_ids:
        target_m = target_process.get_movement_by_movement_id(m_id)
        if use_stored_seed and hasattr(target_m, 'seed'):
            saved_seed = copy(target_m.seed)
        target_m.data = source_process.get_movement_by_movement_id(m_id).data
        if use_stored_seed and hasattr(target_m, 'seed'):
            target_m.seed = saved_seed
