# Cloth and character bodies
> [Previous: Collisions and constraints](02_collisions_and_constraints.md) | [Next: Game optimization](04_game_optimization.md)

---

## 7. Building cloth

A simple cloth mesh can be represented as a grid of particles. Add constraints between neighboring particles. The constraints preserve local spacing, while pins hold selected particles in place.

```text
pin o---o---o---o---o pin
    |   |   |   |   |
    o---o---o---o---o
    |   |   |   |   |
    o---o---o---o---o
```

A grid with `rows` and `columns` has:

- `rows * columns` particles.
- `rows * (columns - 1)` horizontal neighbor constraints.
- `(rows - 1) * columns` vertical neighbor constraints.

The basic grid resists stretching along rows and columns, but it can shear like a parallelogram. Add diagonal **shear constraints** to resist that. Add constraints spanning two or more grid cells, often called **bend constraints**, to resist folding. More constraints make cloth look stiffer, but cost more work and can require more solver passes.

A single pass can produce soft, stretchy cloth. Several passes make the distances more rigid. This gives a useful artistic control as well as a performance control.

### Pinning a particle correctly

A pin can be represented by inverse mass zero. Alternatively, set its position to the target position and set its old position to the same target. Setting both prevents the pin from carrying unintended velocity. If the target itself moves and you want the cloth to inherit that motion, update the old position deliberately instead of always making both positions equal.

### Common cloth problems

- **Exploding or shaking cloth:** check the time step, starting constraint lengths, and zero-distance cases; reduce the step or increase solver passes.
- **Cloth stretches too much:** add solver iterations, substeps, or shear/bend constraints.
- **Cloth looks like a rigid board:** reduce stiffness, remove some long-range constraints, or use fewer iterations.
- **Pinned corners drift:** make pins immovable or reset them after every constraint pass.
- **Cloth falls through a surface:** project particles against the surface during the solver loop and use smaller time steps or continuous/swept collision tests for fast motion.

---

## 8. Rigid bodies made from particles

A 3D rigid body has six degrees of freedom: three for its position and three for its orientation. Jakobsen's paper proposes representing a tetrahedron with four particles and constraining all six edges. The four points provide twelve coordinates; six independent distance constraints leave six degrees of freedom:

$$
4\times 3 - 6 = 6
$$

```text
             p3
            /|\\
           / | \\
          /  |  \\
        p0---|---p2
          \\ |  /
           \\| /
            p1
```

In an implementation, create constraints for every pair of the four particles. Record each edge's initial length as its rest length. During each solver pass, repair collisions and then repair the six lengths. A few passes make the tetrahedron behave approximately rigidly.

This representation avoids explicitly integrating angular velocity, torque, and orientation for the body's core motion. But it has limitations:

- The body is not perfectly rigid when the solver stops after a finite number of passes.
- Four particles make a tetrahedron, not a box-shaped collision surface.
- Particle-only collision tests can miss an intersection when an edge or face passes through geometry but no vertex does.
- The mass distribution and chosen particle layout affect how the body responds.

The paper also suggests other particle arrangements. For example, place particles along three perpendicular axes and constrain both lengths and right angles. The key idea is to choose points and constraints that preserve the shape you want.

---

## 9. Collisions on a body, not just at its particles

To collide a large shape accurately, a collision detector can report:

- A point `p` on the simulated object.
- A point `q` on the obstacle, at the position where contact should occur.
- A direction and penetration depth describing the overlap.

For a stick with endpoints `x1`, `x2`, a point on the stick can be written:

$$
p = c_1x_1 + c_2x_2, \qquad c_1 + c_2 = 1
$$

The coefficients say where the contact lies. If the contact is at endpoint 1, `(c1, c2) = (1, 0)`. If it is one quarter of the way from endpoint 1 to endpoint 2, `(c1, c2) = (0.75, 0.25)`.

Let `delta = q - p`. Moving the endpoints by amounts proportional to their coefficients moves the contact point. For equal particle masses, choose a scalar `lambda` so that the contact reaches `q`:

$$
\lambda = \frac{(q-p)\cdot\delta}{(c_1^2+c_2^2)(\delta\cdot\delta)}
$$

$$
x_1' = x_1 + c_1\lambda\delta, \qquad
x_2' = x_2 + c_2\lambda\delta
$$

For a tetrahedron, use four coefficients, one per vertex. More generally, with contact weights `c_i`, equal masses, and `p = sum(c_i * x_i)`:

$$
\lambda = \frac{(q-p)\cdot\delta}{(\sum_i c_i^2)(\delta\cdot\delta)}, \qquad
x_i' = x_i + c_i\lambda\delta
$$

This derivation assumes the collision system has already found a meaningful contact and that the denominator is nonzero. The distance constraints must be solved again afterward, because moving the body to fix a collision can deform it.

### Unequal masses

If each vertex has inverse mass `w_i`, distribute the correction according to mass:

$$
\lambda = \frac{(q-p)\cdot\delta}{(\sum_i w_i c_i^2)(\delta\cdot\delta)}, \qquad
x_i' = x_i + w_i c_i\lambda\delta
$$

A zero inverse mass means that point does not move. For two dynamic bodies, the contact correction is shared between both bodies according to their inverse masses. This is the same principle as a mass-weighted distance constraint.

### Embedding particles inside a visible shape

The collision shape does not have to match the particle scaffold. A tetrahedron can sit inside a cube or another mesh. For a point on a triangle, compute **barycentric coordinates**: three weights that sum to one and describe the point as a blend of the triangle's vertices. Convert those vertex weights to the scaffold particles' weights, then apply the correction to the scaffold.

This separates two jobs:

- The **render mesh** gives the object its visible shape.
- The **particle scaffold** provides a compact, constraint-driven physical model.

Modern collision systems often use a **broad phase** to quickly find likely object pairs, then a **narrow phase** to compute detailed contact information. Spatial trees and bounding volumes help avoid checking every triangle against every object.

---

## 10. Articulated bodies and character rigs

An **articulated body** is a group of parts connected by joints. In a particle scaffold, parts can share particles:

- Sharing one particle makes a point-like connection, similar to a pin joint.
- Sharing two particles creates a shared axis, similar to a hinge.
- A distance constraint can connect particles that belong to different parts.

Real joints also have limits. A knee should bend mostly one way; elbows should not fold backward indefinitely. The paper describes several ways to express limits as constraints:

- A minimum-distance rule prevents two particles from getting too close.
- An angle rule can be written using a dot product between limb directions.
- A plane constraint keeps a particle on one side of or within a plane.

A dot product compares direction and angle. For vectors `u` and `v`:

$$
u\cdot v = \lVert u\rVert\lVert v\rVert\cos(\theta)
$$

Therefore angle limits can be converted into dot-product limits, with careful handling of vector lengths and the allowed angle range.

Jakobsen's character example uses a simplified stick figure rather than a full rigid body for every limb. Particle positions determine where limbs are; rendering code derives limb orientation from those positions. This skips rotation around a limb's long axis, which can be acceptable for a falling character and much cheaper than simulating every rotational degree of freedom.

### Inverse kinematics (IK)

**Inverse kinematics** means asking the system to place a body part at a target, such as keeping a hand on a doorknob. In this method, pin the hand particle to the target during each relaxation pass. Other connected constraints then pull the arm and body into a compatible pose. An immovable target has inverse mass zero.

This is simple and useful, but it is not a complete animation or joint solver. It may need additional angle limits and careful ordering to produce a natural pose.

---
