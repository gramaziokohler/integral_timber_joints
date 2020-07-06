from integral_timber_joints.tools import Tool

class ToolChanger (Tool):
    def __init__(self, name,
        type_name = "ToolChanger",
        tool_coordinate_frame = None):

        # Call Tool init
        if tool_coordinate_frame is None: tool_coordinate_frame = Frame.worldXY()

        super(ToolChanger, self).__init__(name, type_name, tool_coordinate_frame)
