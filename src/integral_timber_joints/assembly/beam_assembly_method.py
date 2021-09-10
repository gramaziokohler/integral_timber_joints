

class BeamAssemblyMethod(object):
    """
    Values > GROUND_CONTACT imply assembly tools are used.
    """
    UNDEFINED = -1      # UNDEFINED : Default starting value when the method is not computed
    GROUND_CONTACT = 0  # GROUND_CONTACT : no assembly tools.
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
    screw_methods = [SCREWED_WITH_GRIPPER, SCREWED_WITHOUT_GRIPPER]
    readable_names_dict = {
        "GroundContact": GROUND_CONTACT,
        "Clamped": CLAMPED,
        "ScrewedWithGripper": SCREWED_WITH_GRIPPER,
        "ScrewedWithoutGripper": SCREWED_WITHOUT_GRIPPER,
    }

