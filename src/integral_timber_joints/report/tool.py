from collections import Counter

from integral_timber_joints.process import RobotClampAssemblyProcess
from integral_timber_joints.report.beam_report import BeamReport
from integral_timber_joints.rhino.load import get_activedoc_path_no_ext, get_process, process_is_none
from integral_timber_joints.tools import Gripper


def tool_report(process, file_path=None):
    # type: (RobotClampAssemblyProcess, str) -> BeamReport
    from integral_timber_joints.assembly import BeamAssemblyMethod
    assembly = process.assembly

    report = BeamReport("Gripper Report")
    gripper_id_counter = Counter()
    gripper_type_counter = Counter()
    tool_id_counter = Counter()
    tool_type_counter = Counter()

    for beam_id in process.assembly.sequence:
        beam = assembly.beam(beam_id)

        # Check and skip BeamAssemblyMethod.MANUAL_ASSEMBLY
        assembly_method = assembly.get_assembly_method(beam_id)
        if assembly_method == BeamAssemblyMethod.MANUAL_ASSEMBLY:
            report.add_info(beam_id, 'MANUAL_ASSEMBLY, no gripper used.')
            continue

        # * Gripper Information
        gripper_type = assembly.get_beam_attribute(beam_id, "gripper_type")
        gripper_id = assembly.get_beam_attribute(beam_id, "gripper_id")
        report.add_info(beam_id, 'Gripper Type (id): %s (%s)' % (gripper_type, gripper_id))
        gripper_id_counter[gripper_id] += 1
        gripper_type_counter[gripper_type] += 1

        if assembly_method in [BeamAssemblyMethod.CLAMPED] + BeamAssemblyMethod.screw_methods :
            tools_str = " "
            for joint_id in assembly.get_joint_ids_with_tools_for_beam(beam_id):
                tool_id = assembly.get_joint_attribute(joint_id, "tool_id")
                tool_type = assembly.get_joint_attribute(joint_id, "tool_type")
                tool_id_counter[tool_id] += 1
                tool_type_counter[tool_type] += 1
                tools_str += tool_type + " (" + tool_id + ")"
            report.add_info(beam_id, 'Assembly Tool used:' + tools_str)
            continue


        # * Check inconsistancy
        if gripper_type is None:
            report.add_error(beam_id, 'gripper_type is None')
        if gripper_id is None:
            report.add_error(beam_id, 'gripper_id is None')
        gripper_tcp_in_ocf = assembly.get_beam_attribute(beam_id, 'gripper_tcp_in_ocf')
        if gripper_tcp_in_ocf is None:
            report.add_error(beam_id, 'gripper_tcp_in_ocf is None')

        # * Check gripper length recommendation
        if gripper_id is not None:
            gripper = process.tool(gripper_id)  # type: Gripper
            limits = gripper.beam_length_limits
            if beam.length < limits[0] or beam.length > limits[1]:
                report.add_warn(beam_id, 'beam.length (%s) is outside Gripper Limits: [%s , %s]' % (beam.length, limits[0], limits[1]))

    report.attributes['gripper_id_num_of_times_used'] = gripper_id_counter
    report.attributes['gripper_type_num_of_times_used'] = gripper_type_counter
    report.attributes['assembly_tool_id_num_of_times_used'] = tool_id_counter
    report.attributes['assembly_tool_type_num_of_times_used'] = tool_type_counter

    # * Boiler plate functions to save report and print summary
    if file_path is None:
        file_path = get_activedoc_path_no_ext() + "_report_tool.log"

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
        report = tool_report(process)
        for line in report.to_plain_text(process):
            print(line)
