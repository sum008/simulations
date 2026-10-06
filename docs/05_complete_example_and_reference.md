# Complete example and reference
> [Previous: Game optimization](04_game_optimization.md) | [Guide home](../readme.md)

---

## 12. A complete runnable cloth example

This small program uses `pygame-ce`, a fixed time step, position-based Verlet integration, mass-weighted distance constraints, two pinned top corners, gravity, and simple screen-boundary projection. Save it as `cloth_verlet_demo.py` in the project and run it with the project's Python environment.

Install dependency if needed:

```bash
python -m pip install pygame-ce
```

```python
from dataclasses import dataclass

import pygame as pg


@dataclass
class DistanceConstraint:
    a: int
    b: int
    rest_length: float


class Cloth:
    def __init__(self, columns, rows, spacing, origin):
        self.columns = columns
        self.rows = rows
        self.positions = []
        self.previous = []
        self.inverse_masses = []
        self.constraints = []
        origin_x, origin_y = origin

        for row in range(rows):
            for column in range(columns):
                point = pg.Vector2(
                    origin_x + column * spacing,
                    origin_y + row * spacing,
                )
                self.positions.append(point)
                self.previous.append(point.copy())
                self.inverse_masses.append(1.0)

        def index(row, column):
            return row * columns + column

        for row in range(rows):
            for column in range(columns):
                if column + 1 < columns:
                    a = index(row, column)
                    b = index(row, column + 1)
                    self._add_constraint(a, b)
                if row + 1 < rows:
                    a = index(row, column)
                    b = index(row + 1, column)
                    self._add_constraint(a, b)

        self.pin(index(0, 0))
        self.pin(index(0, columns - 1))

    def _add_constraint(self, a, b):
        rest_length = self.positions[a].distance_to(self.positions[b])
        self.constraints.append(DistanceConstraint(a, b, rest_length))

    def pin(self, particle_index):
        self.inverse_masses[particle_index] = 0.0
        self.previous[particle_index] = self.positions[particle_index].copy()

    def step(self, dt, gravity, bounds, solver_iterations=5, damping=0.999):
        left, top, right, bottom = bounds

        for index, position in enumerate(self.positions):
            if self.inverse_masses[index] == 0.0:
                continue

            displacement = (position - self.previous[index]) * damping
            self.previous[index] = position.copy()
            self.positions[index] = position + displacement + gravity * dt * dt

        for _ in range(solver_iterations):
            for constraint in self.constraints:
                self._solve_constraint(constraint)

            for index, position in enumerate(self.positions):
                if self.inverse_masses[index] == 0.0:
                    continue
                if position.x < left:
                    position.x = left
                    self.previous[index].x = left
                elif position.x > right:
                    position.x = right
                    self.previous[index].x = right

                if position.y < top:
                    position.y = top
                    self.previous[index].y = top
                elif position.y > bottom:
                    position.y = bottom
                    self.previous[index].y = bottom

    def _solve_constraint(self, constraint):
        point_a = self.positions[constraint.a]
        point_b = self.positions[constraint.b]
        weight_a = self.inverse_masses[constraint.a]
        weight_b = self.inverse_masses[constraint.b]
        total_weight = weight_a + weight_b

        if total_weight == 0.0:
            return

        delta = point_b - point_a
        distance_squared = delta.length_squared()
        if distance_squared == 0.0:
            return

        distance = distance_squared**0.5
        error_fraction = (distance - constraint.rest_length) / distance
        correction = delta * error_fraction
        point_a += correction * (weight_a / total_weight)
        point_b -= correction * (weight_b / total_weight)

    def draw(self, surface):
        for row in range(self.rows):
            for column in range(self.columns):
                index = row * self.columns + column
                point = self.positions[index]
                if column + 1 < self.columns:
                    neighbor = self.positions[index + 1]
                    pg.draw.line(surface, (126, 176, 184), point, neighbor, 1)
                if row + 1 < self.rows:
                    neighbor = self.positions[index + self.columns]
                    pg.draw.line(surface, (126, 176, 184), point, neighbor, 1)

        for index, position in enumerate(self.positions):
            color = (242, 190, 93) if self.inverse_masses[index] == 0.0 else (232, 239, 226)
            pg.draw.circle(surface, color, position, 3)


def main():
    pg.init()
    width, height = 900, 700
    screen = pg.display.set_mode((width, height))
    pg.display.set_caption("Position-Based Verlet Cloth")
    clock = pg.time.Clock()

    fixed_dt = 1.0 / 120.0
    accumulator = 0.0
    gravity = pg.Vector2(0, 900)
    cloth = Cloth(columns=30, rows=22, spacing=18, origin=(180, 60))
    running = True

    while running:
        frame_time = min(clock.tick(60) / 1000.0, 0.05)
        accumulator += frame_time

        for event in pg.event.get():
            if event.type == pg.QUIT:
                running = False

        while accumulator >= fixed_dt:
            cloth.step(
                fixed_dt,
                gravity,
                bounds=(8, 8, width - 8, height - 8),
                solver_iterations=5,
            )
            accumulator -= fixed_dt

        screen.fill((24, 30, 36))
        cloth.draw(screen)
        pg.display.flip()

    pg.quit()


if __name__ == "__main__":
    main()
```

### Read the example from the inside out

1. Each particle has a current position and a previous position. Their difference is its implicit motion.
2. `step` predicts all movable particles under gravity.
3. Each distance constraint nudges its endpoints toward their original spacing. Inverse mass zero keeps pinned points fixed.
4. The solver repeats those local repairs five times and keeps movable particles inside the screen bounds.
5. The accumulator runs physics at 120 steps per second even if drawing runs at a different rate.

When a particle reaches a screen edge, the example also sets the matching previous-position coordinate to that edge. This removes velocity into the wall, so the boundary has zero bounce. The other coordinate is left alone, so motion along the wall can continue.

This is deliberately a teaching example, not a production collision system. Its boundary test treats particles as points and the program does not wire in self-collision, friction, wind, diagonal cloth constraints, or swept collision detection. [Section 4 in Collisions and constraints](02_collisions_and_constraints.md#4-collision-by-projection) shows circle/capsule self-collision helpers, while [Section 11 in Game optimization](04_game_optimization.md#11-motion-control-friction-and-practical-details) explains the spatial-hash broad phase. Integrating self-collision means generating candidate pairs during solver passes and filtering neighboring cloth particles. Try changing `solver_iterations` or `damping` and observe the difference.

---

## 13. What this method is good at, and where it struggles

### Strengths

- Easy to understand and implement.
- Position corrections handle simple collisions and resting contact in one framework.
- Constraints are easy to add: distance, pins, joint limits, and shape-preserving links.
- A small number of iterations often looks convincing in interactive applications.
- The solver can stop early when the frame budget is tight.
- It is a natural fit for cloth, ropes, simple soft bodies, and stylized character motion.

### Limitations and modern practice

- Finite iterations leave residual stretch or shape error.
- Poor time-step choices, bad initial data, or missing guards can still cause instability.
- Collision detection is a major part of the problem; projection cannot fix contacts it never finds.
- Friction and restitution need explicit treatment through the position history or a more advanced solver.
- A particle scaffold may not reproduce the rotational behavior of a conventional rigid-body simulator.
- Large connected systems may converge slowly and need better ordering, substeps, or more sophisticated solvers.

The paper's enduring contribution is the practical combination: infer motion from position history, repair collisions by projection, and handle connected structure with repeated constraint corrections. Each part makes the others useful.

---

## 14. Glossary of jargon

| Term | Plain-English meaning |
|---|---|
| Acceleration | How quickly velocity changes; for gravity, usually a constant downward vector. |
| AABB | Axis-aligned bounding box: a rectangle or box whose sides stay parallel to the coordinate axes. |
| Articulated body | Multiple body parts connected by joints. |
| Barycentric coordinates | Weights that describe a point inside a triangle as a blend of its three vertices. |
| Bilateral constraint | A rule that must hold as an equality, such as a stick having one exact target length. |
| Broad phase | A quick collision search that finds likely object pairs. |
| BVH / AABB tree | A hierarchy of bounding boxes used to skip groups of objects that cannot collide with a query. |
| Capsule | A line segment expanded by a circular radius; a useful proxy for ropes and limbs. |
| Constraint | A rule that limits particle positions or relationships. |
| Contact | A collision that is touching or resting, rather than only a brief impact. |
| Degrees of freedom | Independent ways a system can move. A free 3D rigid body has six. |
| Damping | A deliberate reduction of motion or energy over time. |
| Gauss-Seidel iteration | An iterative method that uses each constraint's updated positions immediately. |
| Implicit velocity | Velocity-like motion represented by the difference between current and previous positions. |
| Inverse kinematics (IK) | Finding a body pose that places a selected part, such as a hand, at a target. |
| Inverse mass | `1 / mass`; controls how much a particle moves when corrected. Zero means fixed. |
| Jacobi iteration | An iterative method that computes updates from a shared old snapshot. |
| Narrow phase | Detailed collision calculations for a candidate object pair. |
| SAT (separating-axis test) | A convex-shape overlap test that checks whether the shapes are separated along any candidate axis. |
| Self-collision | Collision between different parts of the same simulated object. |
| Segment tree | A tree structure for efficient one-dimensional interval queries; not a general 2D/3D collision solution. |
| Spatial hash | A dictionary-based grid that groups nearby objects into cells for broad-phase collision search. |
| Penetration depth | How far overlapping objects must be separated to reach contact. |
| Projection | Moving an invalid position to the nearest allowed position or surface. |
| Quadtree / octree | A region tree that splits 2D space into four parts or 3D space into eight parts. |
| Rest length | The target distance between two connected particles. |
| Relaxation | Repeatedly repairing local constraints to approach a globally consistent result. |
| Restitution | Bounciness; how much normal motion remains or reverses after impact. |
| Shear constraint | A cloth constraint that resists sideways distortion, commonly along grid diagonals. |
| Substep | A smaller physics update performed within one rendered frame. |
| Sweep-and-prune | A broad-phase method that sorts object intervals along an axis and checks only intervals that overlap. |
| Tunneling | Passing through a thin obstacle between two collision checks. |
| Unilateral constraint | A one-sided rule, such as a point staying outside a surface or two points not getting too close. |
| Verlet integration | A position-based integration scheme that uses current and previous positions. |

---

## 15. Suggested learning path

1. Run the cloth example unchanged and identify the current position, previous position, and fixed time step.
2. Change gravity and damping independently. Notice that damping removes motion while gravity adds it.
3. Change solver passes from 1 to 10. Watch how the cloth's edge lengths change.
4. Add diagonal constraints and compare the cloth's resistance to shearing.
5. Add a floor contact and then implement tangential friction by adjusting the previous position.
6. Build a rope as a line of particles, then a small articulated stick figure.
7. Only after these are working, experiment with contact points on rigid particle scaffolds and faster collision detection.

When debugging, draw particles, constraints, pinned points, contact normals, and penetration depths. Visualizing the solver state usually reveals mistakes faster than staring at equations.

---

## 16. Reference

Jakobsen, Thomas. “Advanced Character Physics.” *Proceedings of the Game Developers Conference*, 2001. The paper describes the particle, Verlet, projection, relaxation, cloth, rigid-body, articulated-body, and friction techniques explained in this chapter.
