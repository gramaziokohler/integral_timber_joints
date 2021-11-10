from collections import Counter

from integral_timber_joints.process import RobotClampAssemblyProcess
from integral_timber_joints.assembly import BeamAssemblyMethod

from integral_timber_joints.report.beam_report import BeamReport
from integral_timber_joints.rhino.load import get_activedoc_path_no_ext, get_process, process_is_none
from integral_timber_joints.tools import Gripper

from math import ceil


def beam_report(process, file_path=None):
    # type: (RobotClampAssemblyProcess, str) -> BeamReport
    from integral_timber_joints.geometry import Joint, JointPolylineLap

    assembly = process.assembly

    report = BeamReport("Gripper Report")
    estimated_density = 5e-7  # 500kg/m3

    beam_size_counter = Counter()
    assembly_type_counter = Counter()
    joint_type_counter = Counter()
    total_weight = 0
    for beam_id in process.assembly.sequence:
        beam = assembly.beam(beam_id)

        # * Information
        length = ceil(beam.length)
        width = round(beam.width)
        height = round(beam.height)
        volume = length * width * height
        size_str = "%i x %i" % (width, height)
        report.add_info(beam_id, 'Size = %s, Length = %s' % (size_str, length))


        estimated_weight = round(estimated_density * volume)
        report.add_info(beam_id, 'Estimated Weight = %skg' % (estimated_weight))
        total_weight += estimated_weight

        assembly_method = assembly.get_assembly_method(beam_id)
        assembly_method_str = BeamAssemblyMethod.value_to_names_dict[assembly_method]
        report.add_info(beam_id, 'Assembly Method = %s' % (assembly_method_str))

        joints = assembly.get_joint_ids_of_beam(beam_id)
        joints_with_already_built_neighbours = assembly.get_joints_of_beam_connected_to_already_built(beam_id)
        report.add_info(beam_id, 'Number of Joints (# to previous elements) = %i (%i)' % (len(joints), len(joints_with_already_built_neighbours)))

        for joint_id in joints_with_already_built_neighbours:
            joint = assembly.joint(joint_id)
            # * Count Joint Type
            joint_type_counter[joint.__class__.__name__] += 1
            # * Check validity of Polyline Lap
            if type(joint) == JointPolylineLap:
                if joint.check_polyline_interior_angle(89.999999) == False:
                    polyline_interior_angles = joint.get_polyline_interior_angles()
                    polyline_interior_angles = [angle for angles in polyline_interior_angles for angle in angles]
                    message = "WARNING : JointPolylineLap (%s-%s) interior angle < 90degs, min = %f" % (joint_id[0], joint_id[1], min(polyline_interior_angles))
                    report.add_error(beam_id, message)

        beam_size_counter[size_str] += 1
        assembly_type_counter[assembly_method_str] += 1

    report.attributes['total_beam_number'] = len(process.assembly.sequence)
    report.attributes['beam_size_counter'] = beam_size_counter
    report.attributes['assembly_type_counter'] = assembly_type_counter
    report.attributes['estimated_density_kg_per_m3'] = "%.1f"% (estimated_density * 1e9)
    report.attributes['estimated_total_weight_kg'] = total_weight
    report.attributes['joint_type_counter_pairs'] = joint_type_counter

    # * Boiler plate functions to save report and print summary
    if file_path is None:
        file_path = get_activedoc_path_no_ext() + "_report_beam.log"

    print("Gripper Report Saved to: %s" % file_path)
    beams_with_warn_or_error = report.beams_with_warn_or_error()
    if beams_with_warn_or_error == []:
        print("- No Errors or Warnings.")
    else:
        print("- The following beams have errors or warnings: %s See report for details" % beams_with_warn_or_error)
    report.to_json(file_path, pretty=True)
    return report


if __name__ == '__main__':
    process = get_process()
    if process_is_none(process):
        print("Load json first")
    else:
        report = beam_report(process)
        for line in report.to_plain_text(process):
            print(line)
