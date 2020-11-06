import os

import compas
import compas_bootstrapper
import compas_fab
import rhinoscriptsyntax as rs

from integral_timber_joints.process import RobotClampAssemblyProcess
from integral_timber_joints.rhino.load import get_process_artist


def print_package_info():
    compas_version = compas.__version__
    compas_path = os.path.abspath(compas.__file__)

    compas_fab_version = compas_fab.__version__
    compas_fab_path = os.path.abspath(compas_fab.__file__)

    conda_env_name = compas_bootstrapper.ENVIRONMENT_NAME
    python_directory = compas_bootstrapper.PYTHON_DIRECTORY

    print('compas_version = %s' % compas_version)
    print('compas_path = %s' % compas_path)
    print('compas_fab_version = %s' % compas_fab_version)
    print('compas_fab_path = %s' % compas_fab_path)
    print('conda_env_name = %s' % conda_env_name)
    print('python_directory = %s' % python_directory)
    return (
        compas_version, compas_path,
        compas_fab_version, compas_fab_path,
        conda_env_name, python_directory,
    )


def recompute_dependent_solutions(process, beam_id):
    # type: (RobotClampAssemblyProcess, str) -> bool
    """ Recompute the downstrem computation of a beam after chaning something upstream.
    """
    assembly = process.assembly
    if assembly.get_beam_attribute(beam_id, 'assembly_wcf_final') is None:
        assembly.compute_beam_assembly_direction_from_joints_and_sequence(beam_id)
    if assembly.get_beam_attribute(beam_id, 'assembly_wcf_inclamp') is None:
        assembly.compute_beam_assembly_direction_from_joints_and_sequence(beam_id)


def get_existing_beams_filter(process, exclude_beam_ids = []):
    # type: (RobotClampAssemblyProcess, list[str]) -> None
    # Create beam guids for filtering
    artist = get_process_artist()
    beam_guids = []
    for beam_id in artist.guids:
        if beam_id not in exclude_beam_ids:
            beam_guids += artist.guids[beam_id]['itj::beams_mesh']
    # print('guids of beams: %s (%i)' % (beam_guids, len(beam_guids)))

    def existing_beams_filter(rhino_object, geometry, component_index):
        if rhino_object.Attributes.ObjectId in beam_guids:
            return True
        return False

    return existing_beams_filter



if __name__ == "__main__":
    print_package_info()
