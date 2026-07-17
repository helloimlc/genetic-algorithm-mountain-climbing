import unittest
import population_all as population
import simulation 
import genome_all as genome
import creature_all as creature
import numpy as np
import csv
import os
import pybullet as p
## === GA DEFAULT PARAMETERS ===
# change parameters accordingly #
POP_SIZE = 40
GENE_COUNT = 3
GENE_COUNT_SYMMETRY = 4
GENERATIONS_B1 = 300
GENERATIONS_B2 = 100
ITERATIONS = 2400

MUTATION_RATE = 0.1
GROW_RATE = 0.1
SHRINK_RATE = 0.25

# Preferred link limits for "good" creatures
# Link limits provide only a negative penalty to the fitness function based on increasing differential, and does not guarantee that creatures spawn within these limits
LINK_LOWER_LIMIT = 6
LINK_UPPER_LIMIT = 12

LINK_SYMMETRY_LOWER_LIMIT = 12
LINK_SYMMETRY_UPPER_LIMIT = 24

## === SIMULATE WITH VISUAL GUI ON? ===
SHOW_GUI = False

## === Folders for results ===
B1_SAVE_FOLDER_DEFAULT = f"B1_results_default_pop{POP_SIZE}_gens{GENERATIONS_B1}_genecount{GENE_COUNT}_mutation{MUTATION_RATE}_shrinkrate{SHRINK_RATE}"
B1_SAVE_FOLDER_POPULATION = "B1_results_vary_population"
B1_SAVE_FOLDER_MUTATION = "B1_results_vary_mutation"
B1_SAVE_FOLDER_GENE_COUNT = "B1_results_vary_gene_count"
B1_SAVE_FOLDER_SHRINK_RATE = "B1_results_vary_shrink_rate"

B1_SAVE_FOLDERS_ALL = [B1_SAVE_FOLDER_DEFAULT,
                    B1_SAVE_FOLDER_POPULATION,
                    B1_SAVE_FOLDER_MUTATION,
                    B1_SAVE_FOLDER_GENE_COUNT,
                    B1_SAVE_FOLDER_SHRINK_RATE]

B2_SAVE_FOLDER_AMP = "B2_results_amplitude"
B2_SAVE_FOLDER_MOTOR_PHASE = "B2_results_motorPHASE"
B2_SAVE_FOLDER_MOTOR_TYPE = "B2_results_motorTYPE"
B2_SAVE_FOLDER_MOTOR_FREQUENCY = "B2_results_frequency"
B2_SAVE_FOLDER_SHAPES = "B2_results_shapes"
B2_SAVE_FOLDER_THIN_LONG = "B2_results_thin_long_links"
B2_SAVE_FOLDER_SINE_ONLY = "B2_results_sine_waveform_only"
B2_SAVE_FOLDER_PULSE_ONLY = "B2_results_pulse_waveform_only"
B2_SAVE_FOLDER_MIXED_WAVEFORMS = "B2_results_mixed_waveform"
B2_SAVE_FOLDER_SYMMETRY = "B2_results_symmetry"

B2_SAVE_FOLDERS_ALL = [B2_SAVE_FOLDER_AMP,
                    B2_SAVE_FOLDER_MOTOR_PHASE,
                    B2_SAVE_FOLDER_MOTOR_TYPE,
                    B2_SAVE_FOLDER_MOTOR_FREQUENCY,
                    B2_SAVE_FOLDER_SHAPES,
                    B2_SAVE_FOLDER_THIN_LONG,
                    B2_SAVE_FOLDER_SINE_ONLY,
                    B2_SAVE_FOLDER_PULSE_ONLY,
                    B2_SAVE_FOLDER_MIXED_WAVEFORMS,
                    B2_SAVE_FOLDER_SYMMETRY]

for folders in B1_SAVE_FOLDERS_ALL + B2_SAVE_FOLDERS_ALL:
    os.makedirs(folders, exist_ok=True)




class TestGA(unittest.TestCase):
    def setUp(self):
        print(f"-----------------Running {self._testMethodName}-----------------")

    def tearDown(self):
        try:
            p.disconnect()
        except Exception:
            pass

    #####################################
    ##########                 ##########
    ##########  Test Cases B1  ##########
    ##########                 ##########
    #####################################
    def B1_testGA_default(self):
        pop = population.Population(pop_size=POP_SIZE, gene_count=GENE_COUNT)
        sim = simulation.Simulation(gui=SHOW_GUI)

        for gen in range(GENERATIONS_B1):

            # Simulate all creatures
            for cr in pop.creatures:    
                sim.run_creature(cr, ITERATIONS)            

            # Retrieve data of fitness and number of links
            fits = [cr.get_climb_fitness(LINK_LOWER_LIMIT, LINK_UPPER_LIMIT) for cr in pop.creatures]
            links = [len(cr.get_expanded_links()) for cr in pop.creatures]

            # Printing out generation number, fittest, mean fitness, mean links, max links
            print(f"Gen {gen}: Fittest = {np.max(fits):.3f}, Mean = {np.mean(fits):.3f}, Mean Links = {np.mean(links):.1f}, Max Links = {np.max(links)}")    
               
            # Create new generation of creatures
            fit_map = population.Population.get_fitness_map(fits)
            new_creatures = []
             
            for i in range(len(pop.creatures)):
                # Parent Selection
                p1_ind = population.Population.select_parent(fit_map)
                p2_ind = population.Population.select_parent(fit_map)
                p1 = pop.creatures[p1_ind]
                p2 = pop.creatures[p2_ind]

                # Crossover DNA, populating the new population with child creatures
                dna = genome.Genome.crossover(p1.dna, p2.dna)
                dna = genome.Genome.point_mutate(dna, rate=MUTATION_RATE, amount=0.25)
                dna = genome.Genome.shrink_mutate(dna, rate=SHRINK_RATE)
                dna = genome.Genome.grow_mutate(dna, rate=GROW_RATE)
                cr = creature.Creature(1,1)
                cr.update_dna(dna)
                new_creatures.append(cr)

            # Elitism: Carry over best creature from previous generation
            max_fit = np.max(fits)
            for cr in pop.creatures:
                if cr.get_climb_fitness(LINK_LOWER_LIMIT, LINK_UPPER_LIMIT) == max_fit:
                    new_cr = creature.Creature(1,1)
                    new_cr.update_dna(cr.dna)
                    new_creatures[0] = new_cr
                    filename = f"{B1_SAVE_FOLDER_DEFAULT}/elite_{gen}.csv"
                    genome.Genome.to_csv(cr.dna, filename)
                    break
            pop.creatures = new_creatures

        self.assertNotEqual(fits[0], 0)
        p.disconnect()

    def B1_testGA_vary_population(self):
        population_values = [20, 60, 80]
        for population_ in population_values:
            print(f"-----------------Running {population_} Population-----------------")
            try:
                p.disconnect()
            except Exception:
                pass
            pop = population.Population(pop_size=population_, gene_count=GENE_COUNT)
            sim = simulation.Simulation(gui=SHOW_GUI)
            
            for gen in range(GENERATIONS_B1):

                # Simulate all creatures
                for cr in pop.creatures:    
                    sim.run_creature(cr, ITERATIONS)            

                # Retrieve data of fitness and number of links
                fits = [cr.get_climb_fitness(LINK_LOWER_LIMIT, LINK_UPPER_LIMIT) for cr in pop.creatures]
                links = [len(cr.get_expanded_links()) for cr in pop.creatures]

                # Printing out generation number, fittest, mean fitness, mean links, max links
                print(f"Gen {gen}: Fittest = {np.max(fits):.3f}, Mean = {np.mean(fits):.3f}, Mean Links = {np.mean(links):.1f}, Max Links = {np.max(links)}")    
                
                # Create new generation of creatures
                fit_map = population.Population.get_fitness_map(fits)
                new_creatures = []
                
                for i in range(len(pop.creatures)):
                    # Parent Selection
                    p1_ind = population.Population.select_parent(fit_map)
                    p2_ind = population.Population.select_parent(fit_map)
                    p1 = pop.creatures[p1_ind]
                    p2 = pop.creatures[p2_ind]

                    # Crossover DNA, populating the new population with child creatures
                    dna = genome.Genome.crossover(p1.dna, p2.dna)
                    dna = genome.Genome.point_mutate(dna, rate=MUTATION_RATE, amount=0.25)
                    dna = genome.Genome.shrink_mutate(dna, rate=SHRINK_RATE)
                    dna = genome.Genome.grow_mutate(dna, rate=GROW_RATE)
                    cr = creature.Creature(1,1)
                    cr.update_dna(dna)
                    new_creatures.append(cr)

                # Elitism: Carry over best creature from previous generation
                max_fit = np.max(fits)
                for cr in pop.creatures:
                    if cr.get_climb_fitness(LINK_LOWER_LIMIT, LINK_UPPER_LIMIT) == max_fit:
                        new_cr = creature.Creature(1,1)
                        new_cr.update_dna(cr.dna)
                        new_creatures[0] = new_cr

                        save_subfolder = os.path.join(B1_SAVE_FOLDER_POPULATION, f"pop_{population_}_gens{GENERATIONS_B1}_genecount{GENE_COUNT}_mutation{MUTATION_RATE}_shrinkrate{SHRINK_RATE}")
                        os.makedirs(save_subfolder, exist_ok=True)
                        filename = os.path.join(save_subfolder, f"elite_{gen}.csv")  

                        genome.Genome.to_csv(cr.dna, filename)
                        break
                pop.creatures = new_creatures
            p.disconnect()

        self.assertNotEqual(fits[0], 0)

    def B1_testGA_vary_mutation(self):
        mutation_values = [0.05, 0.15, 0.2]
        for mutation_ in mutation_values:
            print(f"-----------------Running {mutation_} Mutation-----------------")
            try:
                p.disconnect()
            except Exception:
                pass
            pop = population.Population(pop_size=POP_SIZE, gene_count=GENE_COUNT)
            sim = simulation.Simulation(gui=SHOW_GUI)
            
            for gen in range(GENERATIONS_B1):

                # Simulate all creatures
                for cr in pop.creatures:    
                    sim.run_creature(cr, ITERATIONS)            

                # Retrieve data of fitness and number of links
                fits = [cr.get_climb_fitness(LINK_LOWER_LIMIT, LINK_UPPER_LIMIT) for cr in pop.creatures]
                links = [len(cr.get_expanded_links()) for cr in pop.creatures]

                # Printing out generation number, fittest, mean fitness, mean links, max links
                print(f"Gen {gen}: Fittest = {np.max(fits):.3f}, Mean = {np.mean(fits):.3f}, Mean Links = {np.mean(links):.1f}, Max Links = {np.max(links)}")    
                
                # Create new generation of creatures
                fit_map = population.Population.get_fitness_map(fits)
                new_creatures = []
                
                for i in range(len(pop.creatures)):
                    # Parent Selection
                    p1_ind = population.Population.select_parent(fit_map)
                    p2_ind = population.Population.select_parent(fit_map)
                    p1 = pop.creatures[p1_ind]
                    p2 = pop.creatures[p2_ind]

                    # Crossover DNA, populating the new population with child creatures
                    dna = genome.Genome.crossover(p1.dna, p2.dna)
                    dna = genome.Genome.point_mutate(dna, rate=mutation_, amount=0.25)
                    dna = genome.Genome.shrink_mutate(dna, rate=SHRINK_RATE)
                    dna = genome.Genome.grow_mutate(dna, rate=GROW_RATE)
                    cr = creature.Creature(1,1)
                    cr.update_dna(dna)
                    new_creatures.append(cr)

                # Elitism: Carry over best creature from previous generation
                max_fit = np.max(fits)
                for cr in pop.creatures:
                    if cr.get_climb_fitness(LINK_LOWER_LIMIT, LINK_UPPER_LIMIT) == max_fit:
                        new_cr = creature.Creature(1,1)
                        new_cr.update_dna(cr.dna)
                        new_creatures[0] = new_cr

                        save_subfolder = os.path.join(B1_SAVE_FOLDER_MUTATION, f"pop_{POP_SIZE}_gens{GENERATIONS_B1}_genecount{GENE_COUNT}_mutation{mutation_}_shrinkrate{SHRINK_RATE}")
                        os.makedirs(save_subfolder, exist_ok=True)
                        filename = os.path.join(save_subfolder, f"elite_{gen}.csv")  

                        genome.Genome.to_csv(cr.dna, filename)
                        break
                pop.creatures = new_creatures
            p.disconnect()

        self.assertNotEqual(fits[0], 0)

    def B1_testGA_vary_gene_count(self):
        gene_count_values = [4, 5, 6]
        for gene_count_ in gene_count_values:
            print(f"-----------------Running {gene_count_} Genes-----------------")
            try:
                p.disconnect()
            except Exception:
                pass
            pop = population.Population(pop_size=POP_SIZE, gene_count=gene_count_)
            sim = simulation.Simulation(gui=SHOW_GUI)
            
            for gen in range(GENERATIONS_B1):

                # Simulate all creatures
                for cr in pop.creatures:    
                    sim.run_creature(cr, ITERATIONS)            

                # Retrieve data of fitness and number of links
                fits = [cr.get_climb_fitness(LINK_LOWER_LIMIT, LINK_UPPER_LIMIT) for cr in pop.creatures]
                links = [len(cr.get_expanded_links()) for cr in pop.creatures]

                # Printing out generation number, fittest, mean fitness, mean links, max links
                print(f"Gen {gen}: Fittest = {np.max(fits):.3f}, Mean = {np.mean(fits):.3f}, Mean Links = {np.mean(links):.1f}, Max Links = {np.max(links)}")    
                
                # Create new generation of creatures
                fit_map = population.Population.get_fitness_map(fits)
                new_creatures = []
                
                for i in range(len(pop.creatures)):
                    # Parent Selection
                    p1_ind = population.Population.select_parent(fit_map)
                    p2_ind = population.Population.select_parent(fit_map)
                    p1 = pop.creatures[p1_ind]
                    p2 = pop.creatures[p2_ind]

                    # Crossover DNA, populating the new population with child creatures
                    dna = genome.Genome.crossover(p1.dna, p2.dna)
                    dna = genome.Genome.point_mutate(dna, rate=MUTATION_RATE, amount=0.25)
                    dna = genome.Genome.shrink_mutate(dna, rate=SHRINK_RATE)
                    dna = genome.Genome.grow_mutate(dna, rate=GROW_RATE)
                    cr = creature.Creature(1,1)
                    cr.update_dna(dna)
                    new_creatures.append(cr)

                # Elitism: Carry over best creature from previous generation
                max_fit = np.max(fits)
                for cr in pop.creatures:
                    if cr.get_climb_fitness(LINK_LOWER_LIMIT, LINK_UPPER_LIMIT) == max_fit:
                        new_cr = creature.Creature(1,1)
                        new_cr.update_dna(cr.dna)
                        new_creatures[0] = new_cr

                        save_subfolder = os.path.join(B1_SAVE_FOLDER_GENE_COUNT, f"pop_{POP_SIZE}_gens{GENERATIONS_B1}_genecount{gene_count_}_mutation{MUTATION_RATE}_shrinkrate{SHRINK_RATE}")
                        os.makedirs(save_subfolder, exist_ok=True)
                        filename = os.path.join(save_subfolder, f"elite_{gen}.csv")  

                        genome.Genome.to_csv(cr.dna, filename)
                        break
                pop.creatures = new_creatures
            p.disconnect()

        self.assertNotEqual(fits[0], 0)

    def B1_testGA_vary_shrink_rate(self):
        shrink_rate_values = [0.4, 0.6, 0.8]
        for shrink_rate_ in shrink_rate_values:
            print(f"-----------------Running {shrink_rate_} Shrink Rate-----------------")
            try:
                p.disconnect()
            except Exception:
                pass
            pop = population.Population(pop_size=POP_SIZE, gene_count=GENE_COUNT)
            sim = simulation.Simulation(gui=SHOW_GUI)
            
            for gen in range(GENERATIONS_B1):

                # Simulate all creatures
                for cr in pop.creatures:    
                    sim.run_creature(cr, ITERATIONS)            

                # Retrieve data of fitness and number of links
                fits = [cr.get_climb_fitness(LINK_LOWER_LIMIT, LINK_UPPER_LIMIT) for cr in pop.creatures]
                links = [len(cr.get_expanded_links()) for cr in pop.creatures]

                # Printing out generation number, fittest, mean fitness, mean links, max links
                print(f"Gen {gen}: Fittest = {np.max(fits):.3f}, Mean = {np.mean(fits):.3f}, Mean Links = {np.mean(links):.1f}, Max Links = {np.max(links)}")    
                
                # Create new generation of creatures
                fit_map = population.Population.get_fitness_map(fits)
                new_creatures = []
                
                for i in range(len(pop.creatures)):
                    # Parent Selection
                    p1_ind = population.Population.select_parent(fit_map)
                    p2_ind = population.Population.select_parent(fit_map)
                    p1 = pop.creatures[p1_ind]
                    p2 = pop.creatures[p2_ind]

                    # Crossover DNA, populating the new population with child creatures
                    dna = genome.Genome.crossover(p1.dna, p2.dna)
                    dna = genome.Genome.point_mutate(dna, rate=MUTATION_RATE, amount=0.25)
                    dna = genome.Genome.shrink_mutate(dna, rate=shrink_rate_)
                    dna = genome.Genome.grow_mutate(dna, rate=GROW_RATE)
                    cr = creature.Creature(1,1)
                    cr.update_dna(dna)
                    new_creatures.append(cr)

                # Elitism: Carry over best creature from previous generation
                max_fit = np.max(fits)
                for cr in pop.creatures:
                    if cr.get_climb_fitness(LINK_LOWER_LIMIT, LINK_UPPER_LIMIT) == max_fit:
                        new_cr = creature.Creature(1,1)
                        new_cr.update_dna(cr.dna)
                        new_creatures[0] = new_cr

                        save_subfolder = os.path.join(B1_SAVE_FOLDER_SHRINK_RATE, f"pop_{POP_SIZE}_gens{GENERATIONS_B1}_genecount{GENE_COUNT}_mutation{MUTATION_RATE}_shrinkrate{shrink_rate_}")
                        os.makedirs(save_subfolder, exist_ok=True)
                        filename = os.path.join(save_subfolder, f"elite_{gen}.csv")  

                        genome.Genome.to_csv(cr.dna, filename)
                        break
                pop.creatures = new_creatures
            p.disconnect()

        self.assertNotEqual(fits[0], 0)

    #####################################
    ##########                 ##########
    ##########  Test Cases B2  ##########
    ##########                 ##########
    #####################################
    def B2_testGA_Amplitude(self):
        amplitude_values = [0.5, 1.0, 1.5]
        pop = population.Population(pop_size=POP_SIZE, 
                                    gene_count=GENE_COUNT)
        sim = simulation.Simulation(gui=SHOW_GUI)
        for amplitude_ in amplitude_values:
            print(f"-----------------Running {amplitude_} Amplitude-----------------")
            for gen in range(GENERATIONS_B2):

                # Simulate all creatures
                for cr in pop.creatures:    
                    sim.run_creature(cr, ITERATIONS)            

                # Retrieve data of fitness and number of links
                fits = [cr.get_climb_fitness(LINK_LOWER_LIMIT, LINK_UPPER_LIMIT) for cr in pop.creatures]
                links = [len(cr.get_expanded_links()) for cr in pop.creatures]

                # Printing out generation number, fittest, mean fitness, mean links, max links
                print(f"Gen {gen}: Fittest = {np.max(fits):.3f}, Mean = {np.mean(fits):.3f}, Mean Links = {np.mean(links):.1f}, Max Links = {np.max(links)}")    
                
                # Create new generation of creatures
                fit_map = population.Population.get_fitness_map(fits)
                new_creatures = []
                
                for i in range(len(pop.creatures)):
                    # Parent Selection
                    p1_ind = population.Population.select_parent(fit_map)
                    p2_ind = population.Population.select_parent(fit_map)
                    p1 = pop.creatures[p1_ind]
                    p2 = pop.creatures[p2_ind]

                    # Crossover DNA, populating the new population with child creatures
                    dna = genome.Genome.crossover(p1.dna, p2.dna)
                    dna = genome.Genome.point_mutate(dna, rate=MUTATION_RATE, amount=0.25)
                    dna = genome.Genome.clamp_amp(dna, fixed_amp=amplitude_)  # Clamp amplitude here
                    dna = genome.Genome.shrink_mutate(dna, rate=SHRINK_RATE)
                    dna = genome.Genome.grow_mutate(dna, rate=GROW_RATE)
                    cr = creature.Creature(1,1)
                    cr.update_dna(dna)
                    new_creatures.append(cr)

                # Elitism: Carry over best creature from previous generation
                max_fit = np.max(fits)
                for cr in pop.creatures:
                    if cr.get_climb_fitness(LINK_LOWER_LIMIT, LINK_UPPER_LIMIT) == max_fit:
                        new_cr = creature.Creature(1,1)
                        new_cr.update_dna(cr.dna)
                        new_creatures[0] = new_cr
                        save_subfolder = os.path.join(B2_SAVE_FOLDER_AMP, f"B2_amplitude_{amplitude_}")
                        os.makedirs(save_subfolder, exist_ok=True)
                        filename = f"{save_subfolder}/elite_{gen}.csv"
                        genome.Genome.to_csv(cr.dna, filename)
                        break
                pop.creatures = new_creatures

        self.assertNotEqual(fits[0], 0)
        p.disconnect()

    def B2_testGA_MotorPhase(self):
        
        pop = population.Population(pop_size=POP_SIZE, 
                                    gene_count=GENE_COUNT)
        #sim = simulation.ThreadedSim(pool_size=1)
        sim = simulation.Simulation(gui=SHOW_GUI)

        # Apply randomized motor phase
        for cr in pop.creatures:
            cr.dna = genome.Genome.randomize_phase(cr.dna)

        for gen in range(GENERATIONS_B2):

            # Simulate all creatures
            for cr in pop.creatures:    
                sim.run_creature(cr, ITERATIONS)            

            # Retrieve data of fitness and number of links
            fits = [cr.get_climb_fitness(LINK_LOWER_LIMIT, LINK_UPPER_LIMIT) for cr in pop.creatures]
            links = [len(cr.get_expanded_links()) for cr in pop.creatures]

            # Printing out generation number, fittest, mean fitness, mean links, max links
            print(f"Gen {gen}: Fittest = {np.max(fits):.3f}, Mean = {np.mean(fits):.3f}, Mean Links = {np.mean(links):.1f}, Max Links = {np.max(links)}")    
               
            # Create new generation of creatures
            fit_map = population.Population.get_fitness_map(fits)
            new_creatures = []
             
            for i in range(len(pop.creatures)):
                # Parent Selection
                p1_ind = population.Population.select_parent(fit_map)
                p2_ind = population.Population.select_parent(fit_map)
                p1 = pop.creatures[p1_ind]
                p2 = pop.creatures[p2_ind]

                # Crossover DNA, populating the new population with child creatures
                dna = genome.Genome.crossover(p1.dna, p2.dna)
                dna = genome.Genome.point_mutate(dna, rate=MUTATION_RATE, amount=0.25)
                dna = genome.Genome.shrink_mutate(dna, rate=SHRINK_RATE)
                dna = genome.Genome.grow_mutate(dna, rate=GROW_RATE)
                cr = creature.Creature(1,1)
                cr.update_dna(dna)
                new_creatures.append(cr)

            # Elitism: Carry over best creature from previous generation
            max_fit = np.max(fits)
            for cr in pop.creatures:
                if cr.get_climb_fitness(LINK_LOWER_LIMIT, LINK_UPPER_LIMIT) == max_fit:
                    new_cr = creature.Creature(1,1)
                    new_cr.update_dna(cr.dna)
                    new_creatures[0] = new_cr
                    filename = os.path.join(B2_SAVE_FOLDER_MOTOR_PHASE, f"elite_gen{gen}.csv")
                    genome.Genome.to_csv(cr.dna, filename)
                    break
            
            pop.creatures = new_creatures

        self.assertNotEqual(fits[0], 0)
        p.disconnect()

    def B2_testGA_MotorType(self):
        
        pop = population.Population(pop_size=POP_SIZE, 
                                    gene_count=GENE_COUNT)
        #sim = simulation.ThreadedSim(pool_size=1)
        sim = simulation.Simulation(gui=SHOW_GUI)

        for gen in range(GENERATIONS_B2):

            # Simulate all creatures
            for cr in pop.creatures:    
                sim.run_creature(cr, ITERATIONS)            

            # Retrieve data of fitness and number of links
            fits = [cr.get_climb_fitness(LINK_LOWER_LIMIT, LINK_UPPER_LIMIT) for cr in pop.creatures]
            links = [len(cr.get_expanded_links()) for cr in pop.creatures]

            # Printing out generation number, fittest, mean fitness, mean links, max links
            print(f"Gen {gen}: Fittest = {np.max(fits):.3f}, Mean = {np.mean(fits):.3f}, Mean Links = {np.mean(links):.1f}, Max Links = {np.max(links)}")    
               
            # Create new generation of creatures
            fit_map = population.Population.get_fitness_map(fits)
            new_creatures = []
             
            for i in range(len(pop.creatures)):
                # Parent Selection
                p1_ind = population.Population.select_parent(fit_map)
                p2_ind = population.Population.select_parent(fit_map)
                p1 = pop.creatures[p1_ind]
                p2 = pop.creatures[p2_ind]

                # Crossover DNA, populating the new population with child creatures
                dna = genome.Genome.crossover(p1.dna, p2.dna)
                dna = genome.Genome.point_mutate(dna, rate=MUTATION_RATE, amount=0.25)
                dna = genome.Genome.shrink_mutate(dna, rate=SHRINK_RATE)
                dna = genome.Genome.grow_mutate(dna, rate=GROW_RATE)
                cr = creature.Creature(1,1)
                cr.update_dna(dna)
                new_creatures.append(cr)

            # Elitism: Carry over best creature from previous generation
            max_fit = np.max(fits)
            for cr in pop.creatures:
                if cr.get_climb_fitness(LINK_LOWER_LIMIT, LINK_UPPER_LIMIT) == max_fit:
                    new_cr = creature.Creature(1,1)
                    new_cr.update_dna(cr.dna)
                    new_creatures[0] = new_cr
                    filename = os.path.join(B2_SAVE_FOLDER_MOTOR_TYPE, f"elite_gen{gen}.csv")
                    genome.Genome.to_csv(cr.dna, filename)
                    break
            
            pop.creatures = new_creatures

        self.assertNotEqual(fits[0], 0)
        p.disconnect()

    def B2_testGA_MotorFrequency(self):
        frequency_values = [1.5, 2.0, 2.5]
        pop = population.Population(pop_size=POP_SIZE, 
                                    gene_count=GENE_COUNT)
        #sim = simulation.ThreadedSim(pool_size=1)
        sim = simulation.Simulation(gui=SHOW_GUI)

        for frequency_ in frequency_values:
            print(f"-----------------Running {frequency_} Frequency-----------------")
            for gen in range(GENERATIONS_B2):

                # Simulate all creatures
                for cr in pop.creatures:    
                    sim.run_creature(cr, ITERATIONS)            

                # Retrieve data of fitness and number of links
                fits = [cr.get_climb_fitness(LINK_LOWER_LIMIT , LINK_UPPER_LIMIT) for cr in pop.creatures]
                links = [len(cr.get_expanded_links()) for cr in pop.creatures]

                # Printing out generation number, fittest, mean fitness, mean links, max links
                print(f"Gen {gen}: Fittest = {np.max(fits):.3f}, Mean = {np.mean(fits):.3f}, Mean Links = {np.mean(links):.1f}, Max Links = {np.max(links)}")    
                
                # Create new generation of creatures
                fit_map = population.Population.get_fitness_map(fits)
                new_creatures = []
                
                for i in range(len(pop.creatures)):
                    # Parent Selection
                    p1_ind = population.Population.select_parent(fit_map)
                    p2_ind = population.Population.select_parent(fit_map)
                    p1 = pop.creatures[p1_ind]
                    p2 = pop.creatures[p2_ind]

                    # Crossover DNA, populating the new population with child creatures
                    dna = genome.Genome.crossover(p1.dna, p2.dna)
                    dna = genome.Genome.point_mutate(dna, rate=MUTATION_RATE, amount=0.25)
                    dna = genome.Genome.clamp_freq(dna, fixed_freq=frequency_)
                    dna = genome.Genome.shrink_mutate(dna, rate=SHRINK_RATE)
                    dna = genome.Genome.grow_mutate(dna, rate=GROW_RATE)
                    cr = creature.Creature(1,1)
                    cr.update_dna(dna)
                    new_creatures.append(cr)

                # Elitism: Carry over best creature from previous generation
                max_fit = np.max(fits)
                for cr in pop.creatures:
                    if cr.get_climb_fitness(LINK_LOWER_LIMIT, LINK_UPPER_LIMIT) == max_fit:
                        new_cr = creature.Creature(1,1)
                        new_cr.update_dna(cr.dna)
                        new_creatures[0] = new_cr
                        save_subfolder = os.path.join(B2_SAVE_FOLDER_MOTOR_FREQUENCY, f"B2_frequency_{frequency_}")
                        os.makedirs(save_subfolder, exist_ok=True)
                        filename = f"{save_subfolder}/elite_{gen}.csv"
                        genome.Genome.to_csv(cr.dna, filename)
                        break

                
                pop.creatures = new_creatures

        self.assertNotEqual(fits[0], 0)
        p.disconnect()

    def B2_testGA_Shapes(self):
        
        log_data = []  # store shape + fitness info
        pop = population.Population(pop_size=POP_SIZE, 
                                    gene_count=GENE_COUNT,
                                    cylinder_only=False)
        #sim = simulation.ThreadedSim(pool_size=1)
        sim = simulation.Simulation(gui=SHOW_GUI)

        for gen in range(GENERATIONS_B2):

            # Simulate all creatures
            for cr in pop.creatures:    
                sim.run_creature(cr, ITERATIONS)            

            # Retrieve data of fitness and number of links
            fits = [cr.get_climb_fitness(LINK_LOWER_LIMIT, LINK_UPPER_LIMIT) for cr in pop.creatures]
            links = [len(cr.get_expanded_links()) for cr in pop.creatures]

            top_creature = pop.get_fittest_creature(fits)
            shape_counts = self.count_shapes(top_creature)
            best_fit = np.max(fits)

            # Print to terminal
            print(f"Gen {gen}: Shapes = {shape_counts}")

            # Store for CSV
            log_data.append({
                "generation": gen,
                "cylinders": shape_counts["cylinder"],
                "boxes": shape_counts["box"],
                "spheres": shape_counts["sphere"],
                "best_fitness": best_fit
            })

            # Printing out generation number, fittest, mean fitness, mean links, max links
            print(f"Gen {gen}: Fittest = {np.max(fits):.3f}, Mean = {np.mean(fits):.3f}, Mean Links = {np.mean(links):.1f}, Max Links = {np.max(links)}")    
               
            # Create new generation of creatures
            fit_map = population.Population.get_fitness_map(fits)
            new_creatures = []
             
            for i in range(len(pop.creatures)):
                # Parent Selection
                p1_ind = population.Population.select_parent(fit_map)
                p2_ind = population.Population.select_parent(fit_map)
                p1 = pop.creatures[p1_ind]
                p2 = pop.creatures[p2_ind]

                # Crossover DNA, populating the new population with child creatures
                dna = genome.Genome.crossover(p1.dna, p2.dna)
                dna = genome.Genome.point_mutate(dna, rate=MUTATION_RATE, amount=0.25)
                dna = genome.Genome.shrink_mutate(dna, rate=SHRINK_RATE)
                dna = genome.Genome.grow_mutate(dna, rate=GROW_RATE)
                cr = creature.Creature(1,1, cylinder_only=False)
                cr.update_dna(dna)
                new_creatures.append(cr)

            # Elitism: Carry over best creature from previous generation
            max_fit = np.max(fits)
            for cr in pop.creatures:
                if cr.get_climb_fitness(LINK_LOWER_LIMIT, LINK_UPPER_LIMIT) == max_fit:
                    new_cr = creature.Creature(1, 1, cylinder_only=False)
                    new_cr.update_dna(cr.dna)
                    new_creatures[0] = new_cr
                    filename = os.path.join(B2_SAVE_FOLDER_SHAPES, f"elite_gen{gen}.csv")
                    genome.Genome.to_csv(cr.dna, filename)
                    break
            
            pop.creatures = new_creatures

        self.assertNotEqual(fits[0], 0)
        p.disconnect()

        # Write the CSV AFTER the loop ends
        with open("shape_log.csv", "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=log_data[0].keys())
            writer.writeheader()
            writer.writerows(log_data)

    def B2_testGA_ThinLong(self):
        
        pop = population.Population(pop_size=POP_SIZE, 
                                    gene_count=GENE_COUNT)
        #sim = simulation.ThreadedSim(pool_size=1)
        sim = simulation.Simulation(gui=SHOW_GUI)

        for gen in range(GENERATIONS_B2):

            # Simulate all creatures
            for cr in pop.creatures:    
                sim.run_creature(cr, ITERATIONS)            

            # Retrieve data of fitness and number of links
            fits = [cr.get_climb_fitness(LINK_LOWER_LIMIT, LINK_UPPER_LIMIT) for cr in pop.creatures]
            links = [len(cr.get_expanded_links()) for cr in pop.creatures]

            # Printing out generation number, fittest, mean fitness, mean links, max links
            print(f"Gen {gen}: Fittest = {np.max(fits):.3f}, Mean = {np.mean(fits):.3f}, Mean Links = {np.mean(links):.1f}, Max Links = {np.max(links)}")    
               
            # Create new generation of creatures
            fit_map = population.Population.get_fitness_map(fits)
            new_creatures = []
             
            for i in range(len(pop.creatures)):
                # Parent Selection
                p1_ind = population.Population.select_parent(fit_map)
                p2_ind = population.Population.select_parent(fit_map)
                p1 = pop.creatures[p1_ind]
                p2 = pop.creatures[p2_ind]

                # Crossover DNA, populating the new population with child creatures
                dna = genome.Genome.crossover(p1.dna, p2.dna)
                dna = genome.Genome.point_mutate(dna, rate=MUTATION_RATE, amount=0.25)
                dna = genome.Genome.shrink_mutate(dna, rate=SHRINK_RATE)
                dna = genome.Genome.grow_mutate(dna, rate=GROW_RATE)
                cr = creature.Creature(1,1)
                cr.update_dna(dna)
                new_creatures.append(cr)

            # Elitism: Carry over best creature from previous generation
            max_fit = np.max(fits)
            for cr in pop.creatures:
                if cr.get_climb_fitness(LINK_LOWER_LIMIT, LINK_UPPER_LIMIT) == max_fit:
                    new_cr = creature.Creature(1,1)
                    new_cr.update_dna(cr.dna)
                    new_creatures[0] = new_cr
                    filename = os.path.join(B2_SAVE_FOLDER_THIN_LONG, f"elite_gen{gen}.csv")
                    genome.Genome.to_csv(cr.dna, filename)
                    break
            
            pop.creatures = new_creatures

        self.assertNotEqual(fits[0], 0)
        p.disconnect()

    def B2_testGA_SineOnly(self):
        
        pop = population.Population(pop_size=POP_SIZE, 
                                    gene_count=GENE_COUNT)
        #sim = simulation.ThreadedSim(pool_size=1)
        sim = simulation.Simulation(gui=SHOW_GUI)

        for gen in range(GENERATIONS_B2):

            # Simulate all creatures
            for cr in pop.creatures:    
                sim.run_creature(cr, ITERATIONS)            

            # Retrieve data of fitness and number of links
            fits = [cr.get_climb_fitness(LINK_LOWER_LIMIT, LINK_UPPER_LIMIT) for cr in pop.creatures]
            links = [len(cr.get_expanded_links()) for cr in pop.creatures]

            # Printing out generation number, fittest, mean fitness, mean links, max links
            print(f"Gen {gen}: Fittest = {np.max(fits):.3f}, Mean = {np.mean(fits):.3f}, Mean Links = {np.mean(links):.1f}, Max Links = {np.max(links)}")    
               
            # Create new generation of creatures
            fit_map = population.Population.get_fitness_map(fits)
            new_creatures = []
             
            for i in range(len(pop.creatures)):
                # Parent Selection
                p1_ind = population.Population.select_parent(fit_map)
                p2_ind = population.Population.select_parent(fit_map)
                p1 = pop.creatures[p1_ind]
                p2 = pop.creatures[p2_ind]

                # Crossover and Mutation
                dna = genome.Genome.crossover(p1.dna, p2.dna)
                dna = genome.Genome.point_mutate(dna, rate=MUTATION_RATE, amount=0.25)
                dna = genome.Genome.shrink_mutate(dna, rate=SHRINK_RATE)
                dna = genome.Genome.grow_mutate(dna, rate=GROW_RATE)

                # Force control-waveform to SINE (value > 0.25 and <= 0.5)
                waveform_index = genome.Genome.get_gene_spec()["control-waveform"]["ind"]
                for gene in dna:
                    gene[waveform_index] = 0.3  # Safe value within SINE range

                # Create new creature
                cr = creature.Creature(1, 1)
                cr.update_dna(dna)
                new_creatures.append(cr)

            # Elitism: Carry over best creature from previous generation
            max_fit = np.max(fits)
            for cr in pop.creatures:
                if cr.get_climb_fitness(LINK_LOWER_LIMIT, LINK_UPPER_LIMIT) == max_fit:
                    new_cr = creature.Creature(1,1)
                    new_cr.update_dna(cr.dna)
                    new_creatures[0] = new_cr
                    filename = os.path.join(B2_SAVE_FOLDER_SINE_ONLY, f"elite_{gen}.csv")
                    genome.Genome.to_csv(cr.dna, filename)
                    break
            
            pop.creatures = new_creatures

        self.assertNotEqual(fits[0], 0)
        p.disconnect()

    def B2_testGA_PulseOnly(self):
        
        pop = population.Population(pop_size=POP_SIZE, 
                                    gene_count=GENE_COUNT)
        #sim = simulation.ThreadedSim(pool_size=1)
        sim = simulation.Simulation(gui=SHOW_GUI)

        for gen in range(GENERATIONS_B2):

            # Simulate all creatures
            for cr in pop.creatures:    
                sim.run_creature(cr, ITERATIONS)            

            # Retrieve data of fitness and number of links
            fits = [cr.get_climb_fitness(LINK_LOWER_LIMIT, LINK_UPPER_LIMIT) for cr in pop.creatures]
            links = [len(cr.get_expanded_links()) for cr in pop.creatures]

            # Printing out generation number, fittest, mean fitness, mean links, max links
            print(f"Gen {gen}: Fittest = {np.max(fits):.3f}, Mean = {np.mean(fits):.3f}, Mean Links = {np.mean(links):.1f}, Max Links = {np.max(links)}")    
               
            # Create new generation of creatures
            fit_map = population.Population.get_fitness_map(fits)
            new_creatures = []
             
            for i in range(len(pop.creatures)):
                # Parent Selection
                p1_ind = population.Population.select_parent(fit_map)
                p2_ind = population.Population.select_parent(fit_map)
                p1 = pop.creatures[p1_ind]
                p2 = pop.creatures[p2_ind]

                # Crossover + Mutation
                dna = genome.Genome.crossover(p1.dna, p2.dna)
                dna = genome.Genome.point_mutate(dna, rate=MUTATION_RATE, amount=0.25)
                dna = genome.Genome.shrink_mutate(dna, rate=SHRINK_RATE)
                dna = genome.Genome.grow_mutate(dna, rate=GROW_RATE)

                # Force waveform to PULSE only (range 0.0 to 0.25)
                waveform_index = genome.Genome.get_gene_spec()["control-waveform"]["ind"]
                for gene in dna:
                    gene[waveform_index] = 0.1  # PULSE

                # Create new creature
                cr = creature.Creature(1, 1)
                cr.update_dna(dna)
                new_creatures.append(cr)

            # Elitism: Carry over best creature from previous generation
            max_fit = np.max(fits)
            for cr in pop.creatures:
                if cr.get_climb_fitness(LINK_LOWER_LIMIT, LINK_UPPER_LIMIT) == max_fit:
                    new_cr = creature.Creature(1,1)
                    new_cr.update_dna(cr.dna)
                    new_creatures[0] = new_cr
                    filename = os.path.join(B2_SAVE_FOLDER_PULSE_ONLY, f"elite_{gen}.csv")
                    genome.Genome.to_csv(cr.dna, filename)
                    break
            
            pop.creatures = new_creatures

        self.assertNotEqual(fits[0], 0)
        p.disconnect()

    def B2_testGA_MixedWaveform(self):
        
        pop = population.Population(pop_size=POP_SIZE, 
                                    gene_count=GENE_COUNT)
        #sim = simulation.ThreadedSim(pool_size=1)
        sim = simulation.Simulation(gui=SHOW_GUI)

        for gen in range(GENERATIONS_B2):

            # Simulate all creatures
            for cr in pop.creatures:    
                sim.run_creature(cr, ITERATIONS)            

            # Retrieve data of fitness and number of links
            fits = [cr.get_climb_fitness(LINK_LOWER_LIMIT, LINK_UPPER_LIMIT) for cr in pop.creatures]
            links = [len(cr.get_expanded_links()) for cr in pop.creatures]

            # Printing out generation number, fittest, mean fitness, mean links, max links
            print(f"Gen {gen}: Fittest = {np.max(fits):.3f}, Mean = {np.mean(fits):.3f}, Mean Links = {np.mean(links):.1f}, Max Links = {np.max(links)}")    
               
            # Create new generation of creatures
            fit_map = population.Population.get_fitness_map(fits)
            new_creatures = []
             
            for i in range(len(pop.creatures)):
                # Parent Selection
                p1_ind = population.Population.select_parent(fit_map)
                p2_ind = population.Population.select_parent(fit_map)
                p1 = pop.creatures[p1_ind]
                p2 = pop.creatures[p2_ind]

                # Crossover DNA, populating the new population with child creatures
                dna = genome.Genome.crossover(p1.dna, p2.dna)
                dna = genome.Genome.point_mutate(dna, rate=MUTATION_RATE, amount=0.25)
                dna = genome.Genome.shrink_mutate(dna, rate=SHRINK_RATE)
                dna = genome.Genome.grow_mutate(dna, rate=GROW_RATE)
                dna = genome.Genome.apply_waveform_ratio(dna, sine_ratio=0.7)
                cr = creature.Creature(1,1)
                cr.update_dna(dna)
                new_creatures.append(cr)

            # Elitism: Carry over best creature from previous generation
            max_fit = np.max(fits)
            for cr in pop.creatures:
                if cr.get_climb_fitness(LINK_LOWER_LIMIT, LINK_UPPER_LIMIT) == max_fit:
                    new_cr = creature.Creature(1,1)
                    new_cr.update_dna(cr.dna)
                    new_creatures[0] = new_cr
                    filename = os.path.join(B2_SAVE_FOLDER_MIXED_WAVEFORMS, f"elite_{gen}.csv")
                    genome.Genome.to_csv(cr.dna, filename)
                    break
            
            pop.creatures = new_creatures

        self.assertNotEqual(fits[0], 0)
        p.disconnect()

    def B2_testGA_Symmetry(self):
        
        pop = population.Population(pop_size=POP_SIZE, 
                                    gene_count=GENE_COUNT_SYMMETRY,
                                    symmetric=True)
        #sim = simulation.ThreadedSim(pool_size=1)
        sim = simulation.Simulation(gui=SHOW_GUI)

        for gen in range(GENERATIONS_B2):

            # Simulate all creatures
            for cr in pop.creatures:    
                sim.run_creature(cr, ITERATIONS)            

            # Retrieve data of fitness and number of links
            fits = [cr.get_climb_fitness(LINK_SYMMETRY_LOWER_LIMIT, LINK_SYMMETRY_UPPER_LIMIT) for cr in pop.creatures]
            links = [len(cr.get_expanded_links()) for cr in pop.creatures]

            # Printing out generation number, fittest, mean fitness, mean links, max links
            print(f"Gen {gen}: Fittest = {np.max(fits):.3f}, Mean = {np.mean(fits):.3f}, Mean Links = {np.mean(links):.1f}, Max Links = {np.max(links)}")    
               
            # Create new generation of creatures
            fit_map = population.Population.get_fitness_map(fits)
            new_creatures = []
             
            for i in range(len(pop.creatures)):
                # Parent Selection
                p1_ind = population.Population.select_parent(fit_map)
                p2_ind = population.Population.select_parent(fit_map)
                p1 = pop.creatures[p1_ind]
                p2 = pop.creatures[p2_ind]

                # Crossover DNA, populating the new population with child creatures
                dna = genome.Genome.crossover(p1.dna, p2.dna)
                dna = genome.Genome.point_mutate(dna, rate=MUTATION_RATE, amount=0.25)
                dna = genome.Genome.shrink_mutate(dna, rate=SHRINK_RATE)
                dna = genome.Genome.grow_mutate(dna, rate=GROW_RATE)
                dna = genome.Genome.apply_waveform_ratio(dna, sine_ratio=0.7)
                cr = creature.Creature(1,1)
                cr.update_dna(dna)
                new_creatures.append(cr)

            # Elitism: Carry over best creature from previous generation
            max_fit = np.max(fits)
            for cr in pop.creatures:
                if cr.get_climb_fitness(LINK_SYMMETRY_LOWER_LIMIT, LINK_SYMMETRY_UPPER_LIMIT) == max_fit:
                    new_cr = creature.Creature(1,1)
                    new_cr.update_dna(cr.dna)
                    new_creatures[0] = new_cr
                    filename = os.path.join(B2_SAVE_FOLDER_SYMMETRY, f"elite_{gen}.csv")
                    genome.Genome.to_csv(cr.dna, filename)
                    break
            
            pop.creatures = new_creatures

        self.assertNotEqual(fits[0], 0)
        p.disconnect()

    #####################################
    ########                    #########
    ########  Helper Functions  #########
    ########                    #########
    #####################################
    def count_shapes(self, creature):
        counts = {"cylinder": 0, "box": 0, "sphere": 0}
        for link in creature.get_expanded_links():
            shape_val = link.link_shape
            if shape_val < 0.33:
                counts["cylinder"] += 1
            elif shape_val < 0.66:
                counts["box"] += 1
            else:
                counts["sphere"] += 1
        return counts


if __name__ == "__main__":
    suite = unittest.TestSuite()
    # === uncomment test cases that you want to run === ##
    suite.addTest(TestGA('B1_testGA_default'))
    suite.addTest(TestGA('B1_testGA_vary_population'))
    suite.addTest(TestGA('B1_testGA_vary_mutation'))
    suite.addTest(TestGA('B1_testGA_vary_gene_count'))
    suite.addTest(TestGA('B1_testGA_vary_shrink_rate'))

    suite.addTest(TestGA('B2_testGA_Amplitude'))
    suite.addTest(TestGA('B2_testGA_MotorPhase'))
    suite.addTest(TestGA('B2_testGA_MotorType'))
    suite.addTest(TestGA('B2_testGA_MotorFrequency'))
    suite.addTest(TestGA('B2_testGA_Shapes'))
    suite.addTest(TestGA('B2_testGA_ThinLong'))
    suite.addTest(TestGA('B2_testGA_SineOnly'))
    suite.addTest(TestGA('B2_testGA_PulseOnly'))
    suite.addTest(TestGA('B2_testGA_MixedWaveform'))
    suite.addTest(TestGA('B2_testGA_Symmetry'))

    runner = unittest.TextTestRunner()
    runner.run(suite)


