import genome_all as genome 
from xml.dom.minidom import getDOMImplementation
from enum import Enum
import numpy as np
import random
import pybullet as p

class MotorType(Enum):
    PULSE = 1
    SINE = 2
    TRIANGLE = 3
    SAWTOOTH = 4

class Motor:
    def __init__(self, control_waveform, control_amp, control_freq, control_phase = 0):
        if control_waveform <= 0.25:
            self.motor_type = MotorType.PULSE
        elif control_waveform <= 0.5:
            self.motor_type = MotorType.SINE
        elif control_waveform <= 0.75:
            self.motor_type = MotorType.TRIANGLE
        else:
            self.motor_type = MotorType.SAWTOOTH

        self.phase = control_phase  # Start from custom phase
        self.amp = control_amp
        self.freq = control_freq
    

    def get_output(self):
        self.phase = (self.phase + self.freq) % (np.pi * 2)
        if self.motor_type == MotorType.PULSE:
            output = 1 if self.phase < np.pi else -1
            
        elif self.motor_type == MotorType.SINE:
            output = np.sin(self.phase)

        elif self.motor_type == MotorType.TRIANGLE:
            t = self.phase / (np.pi * 2)  # Normalize to [0,1]
            if t < 0.5:
                output = 4 * t - 1  # -1 to 1
            else:
                output = 3 - 4 * t  # 1 to -1

        elif self.motor_type == MotorType.SAWTOOTH:
            t = self.phase / (np.pi * 2)
            output = 2 * t - 1  # -1 to 1
                
        return output 

class Creature:
    def __init__(self, gene_count, id, permitted_waveforms = (MotorType.PULSE,MotorType.SINE), cylinder_only = True, symmetric = False):
        self.id = id
        self.spec = genome.Genome.get_gene_spec()
        self.symmetric = symmetric

        if self.symmetric:
            self.dna = genome.Genome.get_symmetric_genome(len(self.spec), gene_count)
        else:
            self.dna = genome.Genome.get_random_genome(len(self.spec), gene_count)

        self.flat_links = None
        self.exp_links = None
        self.motors = None
        self.start_position = None
        self.last_position = None
        self.permitted_waveforms = permitted_waveforms
        self.cylinder_only = cylinder_only

    def get_flat_links(self):
        if self.flat_links == None:
            gdicts = genome.Genome.get_genome_dicts(self.dna, self.spec)
            self.flat_links = genome.Genome.genome_to_links(gdicts)
        return self.flat_links
    
    def get_expanded_links(self):
        self.get_flat_links()
        if self.exp_links is not None:
            return self.exp_links
        
        exp_links = [self.flat_links[0]]
        genome.Genome.expandLinks(self.flat_links[0], 
                                self.flat_links[0].name, 
                                self.flat_links, 
                                exp_links)
        self.exp_links = exp_links
        return self.exp_links

    def to_xml(self):
        self.get_expanded_links()
        domimpl = getDOMImplementation()
        adom = domimpl.createDocument(None, "start", None)
        robot_tag = adom.createElement("robot")

        # Step 3: Set flag for shape restriction
        for link in self.exp_links:
            link.link_shape_tag_override = self.cylinder_only

        # Generate link and joint XML
        for link in self.exp_links:
            robot_tag.appendChild(link.to_link_element(adom))
        first = True
        for link in self.exp_links:
            if first:
                first = False
                continue
            robot_tag.appendChild(link.to_joint_element(adom))
        
        robot_tag.setAttribute("name", "pepe")
        return '<?xml version="1.0"?>' + robot_tag.toprettyxml()

    def get_motors(self):
        self.get_expanded_links()
        if self.motors == None:
            motors = []
            for i in range(1, len(self.exp_links)):
                l = self.exp_links[i]
                m = Motor(l.control_waveform, l.control_amp,  l.control_freq, l.control_phase)
                motors.append(m)
            self.motors = motors 
        return self.motors 
    
    def update_position(self, pos):
        if self.start_position == None:
            self.start_position = pos
        else:
            self.last_position = pos

    def get_climb_fitness(self, link_lower_limit, link_upper_limit):
        if self.start_position is None or self.last_position is None:
            return 0.0

        start = np.asarray(self.start_position)
        end = np.asarray(self.last_position)

        # Vertical height climb
        vertical = end[2] - start[2]

        # Horizontal distance travelled towards center
        start_dist = np.linalg.norm(start[:2])  # distance from (0,0) at start
        end_dist = np.linalg.norm(end[:2])      # distance from (0,0) at end
        toward_center = start_dist - end_dist   # positive if it moves closer

        # Generate penalty for too many or too little links
        num_links = len(self.get_expanded_links())
        if num_links < link_lower_limit:
            link_penalty_multiplier = 1 / (link_lower_limit - num_links + 1)
        elif num_links > link_upper_limit:
            link_penalty_multiplier = 1 / (num_links - link_upper_limit + 1)
        else:
            link_penalty_multiplier = 1
        # Strongly prioritize vertical climb, minimal reward for forward motion, scaled according to penalty
        fitness = (0.8 * vertical + 0.2 * toward_center) * (link_penalty_multiplier)
        return max(0.0, fitness)

    def update_dna(self, dna):
        if self.cylinder_only == True:
            spec = genome.Genome.get_gene_spec()
            shape_index = spec["link-shape"]["ind"]
            for gene in dna:
                # Forcing shape index value to be 0.0 (<0.33) so all shapes are cylinder
                gene[shape_index] = 0.0  
        
        if True:
            spec = genome.Genome.get_gene_spec()
            waveform_index = spec["control-waveform"]["ind"]
            waveform_ranges = {
                MotorType.PULSE: (0.00, 0.2499),
                MotorType.SINE: (0.25, 0.4999),
                MotorType.TRIANGLE: (0.50, 0.7499),
                MotorType.SAWTOOTH: (0.75, 0.9999)
            }
            for gene in dna:
                waveform_type = random.choice(self.permitted_waveforms)
                low, high = waveform_ranges[waveform_type]
                gene[waveform_index] = random.uniform(low, high)


        self.dna = dna
        self.flat_links = None
        self.exp_links = None
        self.motors = None
        self.start_position = None
        self.last_position = None

