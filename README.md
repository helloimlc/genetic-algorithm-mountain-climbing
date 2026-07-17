# Genetic Algorithm Mountain Climbing

## Overview

This project implements an evolutionary optimisation system that uses genetic algorithms to evolve virtual creatures capable of climbing a simulated mountain.

The project investigates how different genetic algorithm parameters, creature encoding strategies, and motor control configurations influence the ability of evolved creatures to maximise climbing performance in a physics-based simulation.

---

## Project Objectives

- Integrate a mountain-climbing simulation environment.
- Design an appropriate fitness function for evolutionary optimisation.
- Evolve virtual creatures using a genetic algorithm.
- Investigate the effect of different genetic algorithm parameters.
- Compare different creature encoding and motor control strategies.

---

## Features

- Genetic Algorithm optimisation
- Evolutionary robotics simulation
- Fitness-based creature evolution
- Physics simulation using PyBullet
- Configurable genome representation
- Parameter experimentation
- Performance comparison through experimental analysis

---

## Experiments

The project investigates the impact of different evolutionary parameters, including:

### Basic Genetic Algorithm Parameters

- Population size
- Mutation rate
- Genome size
- Shrink rate

### Advanced Encoding Experiments

- Motor amplitude
- Motor frequency
- Motor phase
- Motor type
- Waveform variations
- Body shapes
- Creature symmetry
- Link configurations

---

## Technologies Used

- Python
- PyBullet
- NumPy
- Genetic Algorithms
- Evolutionary Computation

---

## Repository Structure

```text
genetic-algorithm-mountain-climbing/
│
├── README.md
├── requirements.txt
├── LICENSE
├── .gitignore
│
├── creature.py
├── genome.py
├── population.py
├── simulation.py
├── starter.py
├── prepare_shapes.py
├── realtime_from_csv.py
├── offline_from_csv.py
├── cw_envt.py
│
├── test_creature.py
├── test_genome.py
├── test_population.py
├── test_simulation.py
│
├── shapes/
├── exceptional_extension/
└── *.urdf
```

---

## How to Run

Clone the repository.

Install the required dependencies.

```bash
pip install -r requirements.txt
```

Run the simulation.

```bash
python starter.py
```

---

## Future Improvements

Potential future enhancements include:

- Multi-objective optimisation
- Novelty search
- Reinforcement learning integration
- More complex environments
- Parallel evolution strategies

---

## Author

**Li Ching**
