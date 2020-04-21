from integral_timber_joints.assembly.target import ClampTarget

# Wearhouse Actions

class LoadPickupBeamAction:
    def __init__(self, workpiece):
        self.workpiece = workpiece

    def __str__(self):
        return "Load %s from wearhouse to pickup station" % (self.workpiece)   


# Gripper Actions

class PickToolAction:
    def __init__    (self,  target , tool_type = None,):
        """Action that involves:
        1. Go to Target Approach (Free)
        2. Go to Target Frame (Linear)
        3. Lock tool at quick changer
        4. Go to Target Approach (Linear)

        Parameters
        ----------
        target : [Target]
            [A Target object describing the final location of the ]
        tool_type : [String], optional
            [Type Name of the tool], by default None
        """        
        self.tool_type = tool_type
        self.tool = None
        self.target = target

    def __str__(self):
        if self.tool:
            return "Pick %s from %s" % (self.tool, self.target)   
        else:
            return "Pick Tool (%s) from %s" % (self.tool_type, self.target)    

class PlaceToolAction (PickToolAction):
    ''' Detatch action is similar to attach action
        The only difference is that the tool is unlocked instead of locked.
    '''
    def __str__(self):
        if self.tool:
            return "Place %s to %s" % (self.tool, self.target)   
        else:
            return "Place Tool (%s) to %s" % (self.tool_type, self.target)   

# Workpiece Actions

class PickWorkpieceAction:
    def __init__(self, target, workpiece):
        """Action that involves:
        1. Go to Target Approach (Free)
        2. Go to Target Frame (Linear)
        3. Close tool gripper
        4. Go to Target Retract (Linear)
        """
        self.workpiece = workpiece
        self.target = target
    
    def __str__(self):
        return "Pick %s from %s" % (self.workpiece, self.target)

class PlaceWorkpieceAction(PickWorkpieceAction):
    ''' Place action is similar to attach action
        The only difference is that the gripper finger is opened instead of closed.
    '''
    def __str__(self):
        return "Place %s from %s" % (self.workpiece, self.target)

class DetachWorkpieceAction(PickWorkpieceAction):
    ''' Detatch action only let go of gripper and go to retract
    '''
    def __str__(self):
        return "Detatch %s from %s" % (self.workpiece, self.target)

class LinearMoveWorkpieceAction(PickWorkpieceAction):
    """Action that only involves:
    Go to Target Frame (Linear)
    """
    def __str__(self):
        return "Linear Move %s to %s" % (self.workpiece, self.target)

class FreeMoveWorkpieceAction(PickWorkpieceAction):
    """Action that only involves:
    Go to Target Frame (Free)
    """
    def __str__(self):
        return "Free Move %s to %s" % (self.workpiece, self.target)

# Clamp Actions
      
class ClampJawCloseAction:
    def __init__(self, joint_id):
        """Clamp jaw closing
        
        Parameters
        ----------
        joint_id : (bid1, bid2)
        """        
        self.joint_id = joint_id
        # Target is used by "Assign Tool" algorithm to find assigned tools in scene.
        self.target = ClampTarget(*joint_id)  
        self.tool = None
        
    def __str__(self):
        if self.tool is None:
            return "Clamp(?) at Joint (%s-%s) Clamp-Jaw Close" % self.joint_id
        else:
            return "%s Clamp-Jaw Close" % (self.tool)

class ClampJawOpenAction(ClampJawCloseAction):
    def __str__(self):
        if self.tool is None:
            return "Clamp(?) at Joint (%s-%s) Clamp-Jaw Open" % self.joint_id
        else:
            return "%s Clamp-Jaw Open" % (self.tool)

class ActionGroup:
    def __init__(self):
        self.actions=[]
    def __str__(self):
        if len(self.actions) == 0: return "Empty Action Group"
        if len(self.actions) == 1: return self.actions[0].__str__()
        else: 
            string_builder = ["Action Group with %s actions:" % len(self.actions)]
            for action in self.actions:
                string_builder.append(" -" + action.__str__())
            return '\n'.join(string_builder)
    def append(self, action):
        self.actions.append(action)
    def __iter__(self):
        return self.actions.__iter__()
        