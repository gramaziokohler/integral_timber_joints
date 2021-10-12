

class BeamAssemblyMethod(object):
    """
    Values > GROUND_CONTACT imply assembly tools are used.
    """
    UNDEFINED = -1      # UNDEFINED : Default starting value when the method is not computed
    GROUND_CONTACT = 0  # GROUND_CONTACT : no assembly tools, but still manipulated to place with a robotic gripper.
    # final assembly vector will be hardcoded to be downwards.
    CLAMPED = 1         # CLAMPED : All joints (with earlier neighbours) need a clamp.
    # Clamp information will be stored in joint_attributes.
    # Gripper information is stored in beam_attributes.
    SCREWED_WITH_GRIPPER = 2    # Each joint need a screwdriver.
    # Screwdriver information is stored in joint_attributes.
    # Gripper information is stored in beam_attributes.
    SCREWED_WITHOUT_GRIPPER = 3  # Same as SCREWED_WITH_GRIPPER but one of the screwdriver is used as gripper.
    # Gripper information is stored in beam_attributes but
    # `gripper_type` and `gripper_id` will equal to `tool_type` and `tool_id`
    MANUAL_ASSEMBLY = 4  # No robotic assembly for the beam. It just magically goes to its final pose.

    screw_methods = [SCREWED_WITH_GRIPPER, SCREWED_WITHOUT_GRIPPER]

    value_to_names_dict = {
        UNDEFINED: "UNDEFINED",
        GROUND_CONTACT: "GroundContact",
        CLAMPED: "Clamped",
        SCREWED_WITH_GRIPPER: "ScrewedWithGripper",
        SCREWED_WITHOUT_GRIPPER: "ScrewedWithoutGripper",
        MANUAL_ASSEMBLY: "ManualAssembly",
    }
    names_to_value_dict = {
        "UNDEFINED": UNDEFINED,
        "GroundContact": GROUND_CONTACT,
        "Clamped": CLAMPED,
        "ScrewedWithGripper": SCREWED_WITH_GRIPPER,
        "ScrewedWithoutGripper": SCREWED_WITHOUT_GRIPPER,
        "ManualAssembly": MANUAL_ASSEMBLY,
    }
