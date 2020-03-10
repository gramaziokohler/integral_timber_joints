# Target Classes

from compas.geometry import Translation, Vector

class Target(object):
    def __init__(self):
        """
        All vectors and frames are resolved at Target resolving time.
        All frames are robotic flange (TCP0) frames.
        """        
        self.target_frame = None    # ref to wcf
        self.approach_frame = None  # ref to wcf
        self.approach_vector = None # ref to wcf
        self.retract_frame = None  # ref to wcf
        self.retract_vector = None # ref to wcf
    
    def unresolved(self):
        return (self.target_frame is None or self.approach_frame is None or self.retract_frame is None)

    def resolve(self, target_frame, approach_vector_in_local_coords = None, retract_vector_in_local_coords = None):
        self.target_frame = target_frame
        if (approach_vector_in_local_coords is None):
            approach_vector_in_local_coords = Vector(0,0,0)
        self.approach_vector = target_frame.to_world_coords(approach_vector_in_local_coords)
        T = Translation(self.approach_vector.scaled(-1)) # Reverse the vector 
        self.approach_frame = self.target_frame.transformed(T)
        # If not further defined, retract frame is equal to approach frame
        if (retract_vector_in_local_coords is None):
            self.retract_vector = self.approach_vector.scaled(-1)
            self.retract_frame = self.approach_frame
        else:
            self.retract_vector = target_frame.to_world_coords(retract_vector_in_local_coords)
            self.retract_frame = self.target_frame.transformed(Translation(self.retract_vector))

    def __str__(self):
        return "Target (?)"

# class SingleFrameTarget(object):
#     def __init__(self):
#         self.target_frame = None    # ref to wcf

class StorageTarget(Target):
    ''' Target of a tool or workpiece at storage '''
    def __str__(self):
        if (self.unresolved()): return "UnresolvedStorageTarget"
        else: return "StorageTarget"

class BeamInJawTarget(Target):
    ''' Target to place beam in clamp jaw '''
    def __init__(self, beam):
        Target.__init__(self)
        self.beam = beam

    def __str__(self):
        if (self.unresolved()): return "UnresolvedBeamInJawTarget"
        else: return "BeamInJawTarget"

class BeamAboveJawTarget(Target):
    ''' Target to place beam above clamp jaw '''
    def __init__(self, beam):
        Target.__init__(self)
        self.beam = beam

    def __str__(self):
        if (self.unresolved()): return "UnresolvedBeamAboveJawTarget"
        else: return "BeamAboveJawTarget"


class BeamFinalTarget(Target):
    ''' Beam at its final assembled location '''  
    def __init__(self, beam):
        Target.__init__(self)
        self.beam = beam

    def __str__(self):
        if (self.unresolved()): return "UnresolvedBeamFinalTarget"
        else: return "BeamFinalTarget"

class ClampTarget(Target):
    def __init__(self, beam1, beam2):
        Target.__init__(self)
        self.beam1 = beam1
        self.beam2 = beam2

    def __str__(self):
        if (self.unresolved()): return "UnresolvedClampPosition <%s-%s>" % (self.beam1, self.beam2)
        else: return "ClampTarget"