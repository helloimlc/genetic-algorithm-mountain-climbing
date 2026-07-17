import genome 
from xml.dom.minidom import getDOMImplementation
from enum import Enum
import numpy as np
import pybullet as p

class MotorType(Enum):
    PULSE = 1
    SINE = 2

class Motor:
    def __init__(self, control_waveform, control_amp, control_freq):
        if control_waveform <= 0.5:
            self.motor_type = MotorType.PULSE
        else:
            self.motor_type = MotorType.SINE
        self.amp = control_amp
        self.freq = control_freq
        self.phase = 0
    

    def get_output(self):
        self.phase = (self.phase + self.freq) % (np.pi * 2)
        if self.motor_type == MotorType.PULSE:
            if self.phase < np.pi:
                output = 1
            else:
                output = -1
            
        if self.motor_type == MotorType.SINE:
            output = np.sin(self.phase)
        
        return output 

class Creature:
    def __init__(self, gene_count, id):
        self.id = id
        self.spec = genome.Genome.get_gene_spec()
        self.dna = genome.Genome.get_random_genome(len(self.spec), gene_count)
        self.flat_links = None
        self.exp_links = None
        self.motors = None
        self.start_position = None
        self.last_position = None

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
        for link in self.exp_links:
            robot_tag.appendChild(link.to_link_element(adom))
        first = True
        for link in self.exp_links:
            if first:# skip the root node! 
                first = False
                continue
            robot_tag.appendChild(link.to_joint_element(adom))
        robot_tag.setAttribute("name", "pepe") #  choose a name!
        return '<?xml version="1.0"?>' + robot_tag.toprettyxml()

    def get_motors(self):
        self.get_expanded_links()
        if self.motors == None:
            motors = []
            for i in range(1, len(self.exp_links)):
                l = self.exp_links[i]
                m = Motor(l.control_waveform, l.control_amp,  l.control_freq)
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
        self.dna = dna
        self.flat_links = None
        self.exp_links = None
        self.motors = None
        self.start_position = None
        self.last_position = None

