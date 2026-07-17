import os 
import genome
import sys
import creature
import pybullet as p
import time 
import random
import numpy as np
import simulation
from pathlib import Path
from cw_envt import load_terrain
from cw_envt import load_terrain_hill_and_valleys
from test_ga_no_threads_all import LINK_LOWER_LIMIT, LINK_UPPER_LIMIT, LINK_SYMMETRY_UPPER_LIMIT, LINK_SYMMETRY_LOWER_LIMIT

from test_ga_no_threads_all import B1_SAVE_FOLDER_DEFAULT
from test_ga_no_threads_all import B1_SAVE_FOLDER_POPULATION
from test_ga_no_threads_all import B1_SAVE_FOLDER_MUTATION
from test_ga_no_threads_all import B1_SAVE_FOLDER_GENE_COUNT
from test_ga_no_threads_all import B1_SAVE_FOLDER_SHRINK_RATE

from test_ga_no_threads_all import B2_SAVE_FOLDER_AMP
from test_ga_no_threads_all import B2_SAVE_FOLDER_MOTOR_PHASE
from test_ga_no_threads_all import B2_SAVE_FOLDER_MOTOR_TYPE
from test_ga_no_threads_all import B2_SAVE_FOLDER_MOTOR_FREQUENCY
from test_ga_no_threads_all import B2_SAVE_FOLDER_SHAPES
from test_ga_no_threads_all import B2_SAVE_FOLDER_THIN_LONG
from test_ga_no_threads_all import B2_SAVE_FOLDER_SINE_ONLY
from test_ga_no_threads_all import B2_SAVE_FOLDER_PULSE_ONLY
from test_ga_no_threads_all import B2_SAVE_FOLDER_MIXED_WAVEFORMS
from test_ga_no_threads_all import B2_SAVE_FOLDER_SYMMETRY

## Subfolder Names
NONE = None
GENECOUNT_4 = "pop_40_gens300_genecount4_mutation0.1_shrinkrate0.25"
GENECOUNT_5 = "pop_40_gens300_genecount5_mutation0.1_shrinkrate0.25"
GENECOUNT_6 = "pop_40_gens300_genecount6_mutation0.1_shrinkrate0.25"
MUTATION_0_05 = "pop_40_gens300_genecount3_mutation0.05_shrinkrate0.25"
MUTATION_0_15 = "pop_40_gens300_genecount3_mutation0.15_shrinkrate0.25"
MUTATION_0_20 = "pop_40_gens300_genecount3_mutation0.2_shrinkrate0.25"
POPULATION_20 = "pop_20_gens300_genecount3_mutation0.1_shrinkrate0.25"
POPULATION_60 = "pop_60_gens300_genecount3_mutation0.1_shrinkrate0.25"
POPULATION_80 = "pop_80_gens300_genecount3_mutation0.1_shrinkrate0.25"
SHRINKRATE_0_4 = "pop_40_gens300_genecount3_mutation0.1_shrinkrate0.4"
SHRINKRATE_0_6 = "pop_40_gens300_genecount3_mutation0.1_shrinkrate0.6"
SHRINKRATE_0_8 = "pop_40_gens300_genecount3_mutation0.1_shrinkrate0.8"
B2_AMPLITUDE_0_5 = "B2_amplitude_0.5"
B2_AMPLITUDE_1_0 = "B2_amplitude_1.0"
B2_AMPLITUDE_1_5 = "B2_amplitude_1.5"
B2_FREQUENCY_1_5 = "B2_frequency_1.5"
B2_FREQUENCY_2_0 = "B2_frequency_2.0"
B2_FREQUENCY_2_5 = "B2_frequency_2.5"

FOLDERNAME_MAIN = Path("exceptional_extension") # Main folder
FOLDERNAME_SUB = None      # Sub folder if any, if not initialise as None
# FOLDERNAME_SUB = None
FILENAME = "elite_99.csv"                  # Which generation of creature



def main(csv_file):
    physicsClientId = p.connect(p.GUI)
    p.setPhysicsEngineParameter(enableFileCaching=0)
    p.setGravity(0, 0, -10)
    p.setRealTimeSimulation(1)
    load_terrain()
    # load_terrain_hill_and_valleys()
    p.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW, 0)
    p.configureDebugVisualizer(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW, 0)
    p.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, 0)
    p.resetDebugVisualizerCamera(cameraDistance=7,
                                cameraYaw=50,
                                cameraPitch=-30,
                                cameraTargetPosition=[5,5,0],
                                physicsClientId=physicsClientId)

    load_button = p.addUserDebugParameter(" Load Now", 1, 0, 0)
    prev_button_state = 0

    while True:
        p.stepSimulation()
        button_state = p.readUserDebugParameter(load_button)
        if button_state == 1 and prev_button_state == 0:
            print(f"Loading {csv_file_relative}...")

            # Creature logic
            cr = creature.Creature(gene_count=1, id=1)
            dna = genome.Genome.from_csv(csv_file)
            cr.update_dna(dna)

            with open('test.urdf', 'w') as f:
                f.write(cr.to_xml())

            rob1 = p.loadURDF('test.urdf',[4.5, 4.5, 2.0], [0, 0, 0, 1])

            # Simulate for 30 seconds
            elapsed_time = 0
            wait_time = 1.0 / 240
            sleep_time = 1.0 / 2400
            total_time = 30
            step = 0
            while elapsed_time < total_time:
                p.stepSimulation()
                step += 1
                if step % 24 == 0:
                    motors = cr.get_motors()
                    assert len(motors) == p.getNumJoints(rob1)
                    for jid in range(p.getNumJoints(rob1)):
                        p.setJointMotorControl2(rob1, jid, controlMode=p.VELOCITY_CONTROL, targetVelocity=motors[jid].get_output())

                time.sleep(sleep_time)
                elapsed_time += wait_time
            for jid in range(p.getNumJoints(rob1)):
                p.setJointMotorControl2(rob1, jid, controlMode=p.VELOCITY_CONTROL, targetVelocity=0, force=0)

            fitness_score = cr.get_climb_fitness(LINK_LOWER_LIMIT, LINK_UPPER_LIMIT)
            print("Fitness Score:", fitness_score)
            prev_button_state = button_state



base_dir = Path(__file__).parent

if FOLDERNAME_SUB is not None:
    csv_file_relative = FOLDERNAME_MAIN/FOLDERNAME_SUB/FILENAME
else:
    csv_file_relative = FOLDERNAME_MAIN/FILENAME
csv_file = base_dir/csv_file_relative
main(csv_file)