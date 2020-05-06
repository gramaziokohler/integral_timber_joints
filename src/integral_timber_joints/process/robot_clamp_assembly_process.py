from copy import deepcopy

from compas.geometry import Vector, Translation
from geometric_blocking import blocked
from compas.rpc import Proxy
from integral_timber_joints.assembly import Assembly
from integral_timber_joints.geometry import Joint
from integral_timber_joints.tools import Clamp, Gripper

# I thinkm I need somesort of collision checker, IK reachability checker, Blocking direction checker.

class RobotClampAssemblyProcess:

    def __init__(self, assembly):
        # type(Assembly)
        self.assembly = assembly.copy() # type: Assembly
        self._clamps = {}    # type: Dict[str, Clamp]
        self._grippers = {}  # type: Dict[str, Gripper]

    def add_clamp(self, clamp):
        self._clamps[clamp.name] = clamp.copy()

    def add_gripper(self, gripper):
        self._grippers[gripper.name] = gripper.copy()

    def clamp(self, clamp_name):
        return self._clamps[clamp_name]

    @ property
    def clamps(self):
        return self._clamps.values()

    def gripper(self, gripper_name):
        return self._grippers[gripper_name]

    @ property
    def grippers(self):
        return self._grippers.values()

    @ property
    def available_clamp_types(self):
        return set([clamp.type_name for clamp in self.clamps])

    # -----------------------
    # Clamp
    # -----------------------

    def get_one_clamp_by_type(self, type_name):
        # type: (str) -> Clamp
        """Get one of the clamp that belongs to the given type"""
        for clamp in self.clamps:
            if (clamp.type_name == type_name):
                return clamp
        return None

    def get_clamp_type_of_joint(self, joint_id):
        # type: (Tuple[str, str]) -> str
        """Returns the clamp_type used at the joint. None, if the joint do not need a clamp"""
        if self.assembly.get_joint_attribute(joint_id, 'is_clamp_attached_side'):
            return self.assembly.get_joint_attribute(joint_id, 'clamp_type')

    def get_clamp_of_joint(self, joint_id, set_position_to_clamp_wcf_final=True):
        # type: (Tuple[str, str], bool) -> Clamp
        """Returns one of the clamp object being set at the joint.

        Warning: This clamp object is nor deep-copied.
        Modifying it will change its definition in the process.
        """
        clamp_type = self.get_clamp_type_of_joint(joint_id)
        clamp = self.get_one_clamp_by_type(clamp_type)
        if set_position_to_clamp_wcf_final:
            clamp_wcf_final = self.assembly.get_joint_attribute(joint_id, 'clamp_wcf_final')
            assert clamp_wcf_final is not None
            clamp.set_current_frame_from_tcp(clamp_wcf_final)
        return clamp

    def assign_clamp_type_to_joints(self):
        """Assign clamp_types to joints based on the joint's preference and clamp availability.
        """
        for beam_id in self.assembly.beam_ids():
            # Loop through all the beams and look at their previous_built neighbour.
            for neighbour_id in self.assembly.get_already_built_neighbors(beam_id):
                joint_id = (neighbour_id, beam_id)
                self.assembly.set_joint_attribute(joint_id, "is_clamp_attached_side", True)
                for clamp_type in self.assembly.joint(joint_id).clamp_types:
                    # Check if the preferred clamp exist.
                    if clamp_type in self.available_clamp_types:
                        self.assembly.set_joint_attribute(joint_id, "clamp_type", clamp_type)
                if self.get_clamp_type_of_joint(joint_id) is None:
                    raise RuntimeError("Cannot assign clamp types")
                print ("Joint (%s) assigned clamp_type: %s" % (joint_id , self.assembly.get_joint_attribute(joint_id, "clamp_type")))

    def get_clamp_orientation_options(self, beam_id):
        """Each beam assembly may have multiple clamps involved
        Each clamp may have multiple orientation for attaching to the structure.
        This function returns the orientation possibility for each joint

        returns {'joint_id' : [attachment_frame])}]

        """
        joints_with_clamps_id = [(neighbour_id, beam_id) for neighbour_id in self.assembly.get_already_built_neighbors(beam_id)]

        results = {}
        for joint_id in joints_with_clamps_id:
            joint = self.assembly.joint(joint_id)                   # type: Joint
            neighbour_beam = self.assembly.beam(joint_id[0])
            clamp_attachment_frames = joint.get_clamp_frames(neighbour_beam)  # type: Frame
            results[joint_id] = clamp_attachment_frames
        return results

    def search_valid_clamp_orientation_with_guiding_vector(self, beam_id):
        """ Search and choose clamp attachment frame based on guide vector alignment

        This is applied to all the joints on beam(beam_id) that needs to be clamped.
        where joint(neighbour_id, beam_id)

        Joint attribute 'design_guide_vector_jawapproach')' should be set before hand.
        The -X Axis Vector of the clamp will attempt to align with this vector.
        The vector that is used to place a beam into the jaw.

        This functions sets the joint attribute 'clamp_wcf_final'
        """

        guiding_vector = self.assembly.get_beam_attribute(beam_id, 'design_guide_vector_jawapproach')

        # A Helper function to select best frame.
        def choose_frame_by_guide_vector(frames, guide_vector):
            """ Finds the frame that has the best aligned X axis to the given guide_vector """
            from compas.geometry import dot_vectors
            results = []
            for frame in frames:
                results.append((dot_vectors(guiding_vector, frame.xaxis.scaled(-1.0)), frame))
            guide_score, best_frame = sorted(results, key=lambda x: x[0])[-1] #Lst item of the sorted list has the best
            return best_frame

        # Loop through all the clamp_orientation_options
        chosen_frames = []
        for joint_id, attachment_frames in self.get_clamp_orientation_options(beam_id).items():
            selected_frame = choose_frame_by_guide_vector(attachment_frames, guiding_vector)
            self.assembly.set_joint_attribute(joint_id, 'clamp_wcf_final', selected_frame.copy())
            chosen_frames.append(selected_frame.copy())
            #print ("Beam (%s) Joint (%s), we need clamp type (%s) at %s" % (beam_id, joint_id, self.get_clamp_type_of_joint(joint_id), selected_frame))
        return chosen_frames

    def get_clamp_ids_for_beam(self, beam_id):
        # type: (str) -> Iterator[Tuple[str, str]]
        for neighbour_id in self.assembly.get_already_built_neighbors(beam_id):
            joint_id = (neighbour_id, beam_id)
            if self.assembly.get_joint_attribute(joint_id, 'is_clamp_attached_side') == True:
                yield joint_id

    def compute_jawapproach_vector_length(self, beam_id, vector_dir, min_dist = 20, max_dist = 150):
        # type: (str, Vector) -> Vector
        """Return a `assembly_vector_jawapproach` with correct length that will clear the clamp jaws
        Current implementation do not tkae into account of the angle of the clamp.
        Need to be improved when angled lap joints exist.
        """
        # Compute the length of the movement to clear the clamp jaw
        clearance_length = min_dist
        for joint_id in self.get_clamp_ids_for_beam(beam_id):
            clamp = self.get_clamp_of_joint(joint_id, set_position_to_clamp_wcf_final = True)
            clearance_length = max(clearance_length, clamp.jaw_clearance_vectors_in_wcf.length)
        clearance_length = min(max_dist, clearance_length)

        return vector_dir.unitized().scaled(clearance_length)

    def compute_jawapproach_vector_from_guide_vector_dir(self, beam_id):
        # type: (str) -> Vector
        """Return a `assembly_vector_jawapproach` that follows the direction of `design_guide_vector_jawapproach`
        """
        # Get the guide Vector from beam_attribute
        design_guide_vector_jawapproach = self.assembly.get_beam_attribute(beam_id, 'design_guide_vector_jawapproach')
        assert design_guide_vector_jawapproach is not None

        # Check to see if the guide is blocked by any of the clamp jaws
        for joint_id in self.get_clamp_ids_for_beam(beam_id):
            clamp = self.get_clamp_of_joint(joint_id, set_position_to_clamp_wcf_final = True)
            if blocked(clamp.jaw_blocking_vectors_in_wcf, design_guide_vector_jawapproach.scaled(-1.0)):
                return None

        # Compute the vector length to clear jaw, and return the `assembly_vector_jawapproach`
        return self.compute_jawapproach_vector_length(beam_id, design_guide_vector_jawapproach)

    def compute_jawapproach_vector_from_clamp_jaw_non_blocking_region(self, beam_id):
        # type: (str) -> Vector
        """Return a `assembly_vector_jawapproach` that follows the direction of `design_guide_vector_jawapproach`
        Returns None if the clamps do not present a feasible region.
        """

        # Package to compute Feasible Region
        geometric_blocking = Proxy(package='geometric_blocking')

        # Collect blocking vectors from attached clamps
        blocking_vectors = []
        for joint_id in self.get_clamp_ids_for_beam(beam_id):
            clamp = self.get_clamp_of_joint(joint_id, set_position_to_clamp_wcf_final = True)
            blocking_vectors += clamp.jaw_blocking_vectors_in_wcf

        # Rounding error fix for blocking vectors
        ROUNDING_DIGITS = 10
        blocking_vectors = [Vector(round(x,ROUNDING_DIGITS),round(y,ROUNDING_DIGITS),round(z,ROUNDING_DIGITS)) for x,y,z in blocking_vectors]

        # Compute the analytical result of the feasible region and take a average from the resulting region
        feasible_disassem_rays, lin_set = geometric_blocking.compute_feasible_region_from_block_dir(blocking_vectors)
        if len(feasible_disassem_rays) == 0 : return None
        # # Remove the feasible_rays that are linear (bi-directional)
        feasible_disassem_rays = [Vector(*ray) for (index, ray) in enumerate(feasible_disassem_rays) if (index not in lin_set)]
        feasible_rays_averaged = Vector.sum_vectors(feasible_disassem_rays)

        # Compute the vector length to clear jaw, and return the `assembly_vector_jawapproach`
        return self.compute_jawapproach_vector_length(beam_id, feasible_rays_averaged.scaled(-1))

    def search_valid_jawapproach_vector_prioritizing_guide_vector(self, beam_id):
        # type: (str) -> Vector
        """Search for the `assembly_vector_jawapproach` and `assembly_wcf_inclampapproach`.
        1st priority is to follow direction of `design_guide_vector_jawapproach`.
        2nd priority is to use a vector in the middle of the feisible non-blocking region from the clamp jaw.
        Returns None if no valid vector can be found or no clamps are attached.
        """
        # Exit if no clamp is needed to assemble this beam.
        if len(list(self.get_clamp_ids_for_beam(beam_id))) == 0 : return None

        # Compute vector with different strategies.
        # Each strategy function must return None if no valid solution is found.
        assembly_vector_jawapproach = self.compute_jawapproach_vector_from_guide_vector_dir(beam_id)
        if assembly_vector_jawapproach is None:
            assembly_vector_jawapproach = self.compute_jawapproach_vector_from_clamp_jaw_non_blocking_region(beam_id)
        if assembly_vector_jawapproach is None:
            return None

        # Save result in beam_attribute
        self.assembly.set_beam_attribute(beam_id, 'assembly_vector_jawapproach', assembly_vector_jawapproach)

        # Compute 'ssembly_wcf_inclampapproach' from reversing 'assembly_vector_jawapproach'
        assembly_wcf_inclamp = self.assembly.get_beam_attribute(beam_id, 'assembly_wcf_inclamp')
        T = Translation(assembly_vector_jawapproach.scaled(-1))
        assembly_wcf_inclampapproach = assembly_wcf_inclamp.transformed(T)
        self.assembly.set_beam_attribute(beam_id, 'assembly_wcf_inclampapproach', assembly_wcf_inclampapproach)

        return assembly_vector_jawapproach

    # -----------------------
    # Gripper
    # -----------------------

    def get_one_gripper_by_type(self, type_name):
        # type: (str) -> Gripper
        """Get one of the clamp that belongs to the given type"""
        for gripper in self.grippers:
            if (gripper.type_name == type_name):
                return gripper
        return None

    def copy(self):
        return deepcopy(self)
