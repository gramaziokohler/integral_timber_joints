import os
import time
import logging
import argparse
from typing import Tuple, List, Type, Dict



from integral_timber_joints.planning.parsing import parse_process, save_process, save_movements, get_process_path, rfl_setup
from integral_timber_joints.planning.utils import LOGGER

from integral_timber_joints.process import RoboticFreeMovement, RoboticLinearMovement, RoboticClampSyncLinearMovement, RobotScrewdriverSyncLinearMovement, Movement
from integral_timber_joints.process.movement import RoboticMovement


##############################################
# HERE = os.path.dirname(__file__)

# EXTERNAL_DIR = os.path.abspath(os.path.join(HERE, '..', '..', '..', 'external'))
# DESIGN_STUDY_DIR = os.path.abspath(os.path.join(EXTERNAL_DIR, 'itj_design_study'))

# def get_process_path(design_dir, assembly_name, subdir='.'):
#     if assembly_name.endswith('.json'):
#         filename = os.path.basename(assembly_name)
#     else:
#         filename = '{}.json'.format(assembly_name)
#     folder_dir = os.path.abspath(os.path.join(DESIGN_STUDY_DIR, design_dir, subdir))
#     model_path = os.path.join(folder_dir, filename)
#     mkdir(folder_dir)
#     return model_path

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--design_dir', default='210605_ScrewdriverTestProcess',
                        help='problem json\'s containing folder\'s name.')
    parser.add_argument('--problem', default='nine_pieces_process.json', # twelve_pieces_process.json
                        help='The name of the problem to solve (json file\'s name, e.g. "nine_pieces_process.json")')
    parser.add_argument('--problem_subdir', default='results',
                        help='subdir for saving movements, default to `results`.')
    #
    parser.add_argument('--movement_id', default=None, type=str, help='Compute only for movement with a specific tag, e.g. `A54_M0`. This will force solve_mode to `movement_id` mode and plan for the residing group.')
    #
    parser.add_argument('--debug', action='store_true', help='Debug mode.')
    parser.add_argument('--no_fk', action='store_true', help='Skip FK distance check'
    )
    args = parser.parse_args()


    log_folder = os.path.dirname(get_process_path(args.design_dir, args.problem, subdir=args.problem_subdir))
    log_path = os.path.join(log_folder, 'statistics.log')

    logging_level = logging.DEBUG if args.debug else logging.INFO
    LOGGER.setLevel(logging_level)

    file_handler = logging.FileHandler(filename=log_path, mode='w')
    formatter = logging.Formatter('%(asctime)s | %(name)s | %(levelname)s | %(message)s')
    file_handler.setFormatter(formatter)
    file_handler.setLevel(logging_level)
    LOGGER.addHandler(file_handler)
    LOGGER.info("planning.statistics.py started with args: %s" % args)

    # * Load process
    process = parse_process(args.design_dir, args.problem, subdir=args.problem_subdir)
    result_path = get_process_path(args.design_dir, args.problem, subdir=args.problem_subdir)
    ext_movement_path = os.path.dirname(result_path)

    # * Overall information
    movements = process.movements
    robotic_movements = [m for m in movements if isinstance(m, RoboticMovement)]
    robotic_linear_movements = [m for m in movements if isinstance(m, RoboticLinearMovement)]
    robotic_free_movements = [m for m in movements if isinstance(m, RoboticFreeMovement)]
    LOGGER.info("-"*50)
    LOGGER.info("Process contains %i Movements, %i of which are RoboticMovements" % (len(movements), len(robotic_movements)))
    LOGGER.info("  > among them are %i RoboticLinearMovement and %i RoboticFreeMovement" % (len(robotic_linear_movements), len(robotic_free_movements)))

    # * load previously planned movements
    process.load_external_movements(ext_movement_path)

    # * Trajectory
    robotic_linear_movements_with_traj = [m for m in robotic_linear_movements if m.trajectory is not None]
    LOGGER.info("  > %i of %i RoboticLinearMovement contains trajectory." % (len(robotic_linear_movements_with_traj), len(robotic_linear_movements)))
    robotic_free_movements_with_traj = [m for m in robotic_free_movements if m.trajectory is not None]
    LOGGER.info("  > %i of %i RoboticFreeMovement contains trajectory." % (len(robotic_free_movements_with_traj), len(robotic_free_movements)))

    # * FK Statistics
    LOGGER.info("-"*50)
    for i, movement in enumerate(movements):
        if not isinstance(movement, RoboticMovement):
            continue
        type_tag = "F" if isinstance(movement, RoboticFreeMovement) else "L"
        LOGGER.info("#%4i %8s  %s  %s" % (i, movement.movement_id, type_tag, movement.tag))

        trajectory = movement.trajectory
        if trajectory is None:
            LOGGER.info(" " * 19 + "NO TRAJECTORY PLANNED")
            continue

        if args.no_fk:
            LOGGER.info(" " * 19 + "%-3i TrajPts." % (len(trajectory.points)))
        else:
            flange_frame = None
            total_distance = 0
            for config in trajectory.points:
                configuration = process.robot_initial_config.merged(config)
                trajectory_frame = process.robot_model.forward_kinematics(configuration.scaled(1000), process.ROBOT_END_LINK)
                if flange_frame is not None:
                    total_distance += flange_frame.point.distance_to_point(trajectory_frame.point)
                flange_frame = trajectory_frame

            LOGGER.info(" " * 19 + "%-3i TrajPts, %.1f mm Flange Distance" % (len(trajectory.points), total_distance))

    # * Trajectory
    LOGGER.info("-"*50)
    m_ids = [m.movement_id for m in robotic_linear_movements if m.trajectory is None]
    LOGGER.info("List of RoboticLinearMovement IDs without Trajectory (count = %s):" % (len(m_ids)))
    LOGGER.info("%s" % m_ids)

    m_ids = [m.movement_id for m in robotic_free_movements if m.trajectory is None]
    LOGGER.info("List of RoboticFreeMovement IDs without Trajectory (count = %s):" % (len(m_ids)))
    LOGGER.info("%s" % m_ids)

    m_ids = [m.movement_id for m in robotic_movements if m.trajectory is None]
    LOGGER.info("List of all Movements IDs without Trajectory (count = %s):" % (len(m_ids)))
    LOGGER.info("%s" % m_ids)


    LOGGER.info("-"*50)
    LOGGER.info('End of Statistics.')

if __name__ == '__main__':
    main()
