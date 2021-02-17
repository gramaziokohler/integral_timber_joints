import os

import compas
import compas_bootstrapper
import compas_fab
import rhinoscriptsyntax as rs
import scriptcontext as sc  # type: ignore

from integral_timber_joints.process import RobotClampAssemblyProcess


# This is a duplicate from integral_timber_joints.rhino.load to avoid circular referencing
def get_process_artist():
    if "itj_process_artist" in sc.sticky:
        return sc.sticky["itj_process_artist"]
    else:
        return None


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


def get_existing_beams_filter(process, exclude_beam_ids=[]):
    # type: (RobotClampAssemblyProcess, list[str]) -> None
    # Create beam guids for filtering
    artist = get_process_artist()
    beam_guids = []
    for beam_id in process.assembly.beam_ids():
        if beam_id not in exclude_beam_ids:
            beam_guids.extend(artist.interactive_beam_guid(beam_id))
    # print('guids of beams: %s (%i)' % (beam_guids, len(beam_guids)))

    def existing_beams_filter(rhino_object, geometry, component_index):
        if rhino_object.Attributes.ObjectId in beam_guids:
            return True
        return False

    return existing_beams_filter


# Importing sc functions for purge_objects
try:
    purge_object = sc.doc.Objects.Purge
except AttributeError:
    purge_object = None

find_object = sc.doc.Objects.Find


def purge_objects(guids, redraw=True):
    """Purge objects from memory.
    Adapted from compas_rhino.utilities

    Parameters
    ----------
    guids : list of GUID
    """
    if not purge_object:
        raise RuntimeError('Cannot purge outside Rhino script context')
    rs.EnableRedraw(False)
    for guid in guids:
        if rs.IsObject(guid):
            if rs.IsObjectHidden(guid):
                rs.ShowObject(guid)
            o = find_object(guid)
            purge_object(o.RuntimeSerialNumber)
    if redraw:
        rs.EnableRedraw(True)
        sc.doc.Views.Redraw()


if __name__ == "__main__":
    print_package_info()
