# Advanced Character Physics, Explained

A practical, beginner-friendly guide to Thomas Jakobsen's particle-based physics method.

This guide explains the ideas in the 2001 paper *Advanced Character Physics* using simple language, diagrams, animated examples, equations, and Python code. It starts with Verlet integration and builds toward cloth, collisions, articulated bodies, and game-performance choices.

![Looping animation of Verlet prediction, boundary projection, and circle separation](docs/animations/physics_concepts.svg)

## Read the guide

1. [Verlet fundamentals](docs/01_verlet_fundamentals.md): particles, stored position history, integration, and fixed time steps.
2. [Collisions and constraints](docs/02_collisions_and_constraints.md): projection, self-collision, distance constraints, relaxation, and rigid-body contact.
3. [Cloth and character bodies](docs/03_cloth_and_character_bodies.md): cloth grids, particle-based rigid bodies, and articulated characters.
4. [Game optimization](docs/04_game_optimization.md): broad-phase collision search, segment trees, spatial hashes, other structures, selection guidance, and Python examples.
5. [Complete example and reference](docs/05_complete_example_and_reference.md): a runnable `pygame-ce` cloth simulation, limitations, glossary, and learning path.

## Suggested route

Read chapters 1 and 2 first; they explain the position and collision rules used by the later chapters. Chapter 3 applies those rules to cloth and characters. Chapter 4 compares optimization techniques and shows when each is a sensible choice. Chapter 5 puts the ideas together in a complete program.

The runnable cloth demo uses `pygame-ce`, already listed in this project's requirements. Install it in the project environment if needed:

```bash
python -m pip install pygame-ce
```

Each chapter links back here and forward to the next chapter. The animated SVG assets are stored in `docs/animations/` and linked from the chapters that explain them.
