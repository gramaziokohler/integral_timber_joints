from collections import Counter
from integral_timber_joints.rhino.load import get_activedoc_path_no_ext, get_process, process_is_none
from integral_timber_joints.report.beam_report import BeamReport
from integral_timber_joints.assembly import BeamAssemblyMethod
from integral_timber_joints.process import RobotClampAssemblyProcess


def screw_report(process, file_path=None):
    # type: (RobotClampAssemblyProcess, str) -> BeamReport
    assembly = process.assembly

    report = BeamReport("Screw Report")
    screw_counter = Counter()
    beam_with_screw_counter = Counter()
    screw_thickness_counter = Counter()
    for beam_id in process.assembly.sequence:
        beam = assembly.beam(beam_id)

        # * Information
        assembly_method = assembly.get_assembly_method(beam_id)
        if assembly_method not in BeamAssemblyMethod.screw_methods:
            report.add_info(beam_id, 'Beam is not assembled by screws.')
            beam_with_screw_counter['beam_no_screws'] += 1
            continue
        else:
            beam_with_screw_counter['beam_with_screws'] += 1

        # * Looping over joints from the MovingSide to the StayingSide
        joint_ids = assembly.get_joints_of_beam_connected_to_already_built(beam_id)
        for joint_id in joint_ids:
            tool_type = assembly.get_joint_attribute(joint_id, 'tool_type')
            tool_id = assembly.get_joint_attribute(joint_id, 'tool_id')
            screw = assembly.get_screw_of_joint(joint_id)
            joint = assembly.joint(joint_id)
            moving_side_thickness = joint.thickness
            screw_thickness_counter[screw.name + ' ms_thickness ' + '%2.1fmm' % moving_side_thickness ] += 1
            report.add_info(beam_id, 'Joint (%s) uses Screw: %s MovingSideThickness: %2.1fmm' % (joint_id, screw.name, moving_side_thickness))

            # * Check inconsistancy
            if screw is None:
                report.add_error(beam_id, 'Joint (%s): screw is None' % (joint_id))
                continue

            if not screw.valid:
                report.add_warn(beam_id, 'Joint (%s): Screw (%s) is not valid' % (joint_id, screw.name))

            screw_counter[screw.name] += 1

    # Screw counting
    report.attributes['screw_counter'] = screw_counter
    report.attributes['beam_with_screw_counter'] = beam_with_screw_counter
    report.attributes['screw_thickness_counter'] = screw_thickness_counter

    # * Boiler plate functions to save report and print summary
    if file_path is None:
        file_path = get_activedoc_path_no_ext() + "_report_screw.log"

    print("Screw Report Saved to: %s" % file_path)
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
        report = screw_report(process)
        for line in report.to_plain_text(process):
            print(line)
