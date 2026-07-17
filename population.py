import creature
import numpy as np

class Population:
    def __init__(self, pop_size, gene_count):
        self.creatures = [
            creature.Creature(gene_count=gene_count, id=id)
            for id in range(pop_size)
        ]

    @staticmethod
    def get_fitness_map(fits):
        fitmap = []
        total = 0
        for f in fits:
            total += f
            fitmap.append(total)
        return fitmap

    @staticmethod
    def select_parent(fitmap):
        r = np.random.rand() * fitmap[-1]  # Scale r to total fitness
        for i, f in enumerate(fitmap):
            if r <= f:
                return i

    def get_fittest_creature(self, fits):
        """Returns the creature with the highest fitness."""
        max_index = np.argmax(fits)
        return self.creatures[max_index]
