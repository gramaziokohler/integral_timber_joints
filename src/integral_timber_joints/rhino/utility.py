from integral_timber_joints.geometry import beam
import rhinoscriptsyntax as rs
import compas
import compas_fab
import os
import compas_bootstrapper
from integral_timber_joints.process import RobotClampAssemblyProcess

def print_package_info():
    compas_version  = compas.__version__
    compas_path = os.path.abspath(compas.__file__)

    compas_fab_version = compas_fab.__version__
    compas_fab_path = os.path.abspath(compas_fab.__file__)

    conda_env_name = compas_bootstrapper.ENVIRONMENT_NAME
    python_directory = compas_bootstrapper.PYTHON_DIRECTORY

    print ('compas_version = %s' % compas_version)
    print ('compas_path = %s' % compas_path)
    print ('compas_fab_version = %s' % compas_fab_version)
    print ('compas_fab_path = %s' % compas_fab_path)
    print ('conda_env_name = %s' % conda_env_name)
    print ('python_directory = %s' % python_directory)
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
        print ('meow1')
    if assembly.get_beam_attribute(beam_id, 'assembly_vector_final') is None:
        assembly.compute_beam_assembly_direction_from_joints_and_sequence(beam_id)
        print ('meow2')


if __name__ == "__main__":
    print_package_info()