# from dataclasses import dataclass
import pygame as pg

class Particle:
    def __init__(self, position: pg.Vector2, old_position: pg.Vector2, gravity: pg.Vector2, dt: float, radius: float,
                 damping: float = 1, mass: float = 1, momentum_loss: float = 1, wind: pg.Vector2=pg.Vector2(0,0)):
        self.position: pg.Vector2 = position
        self.old_position: pg.Vector2 = old_position
        self.gravity = gravity
        self.wind = wind
        self.dt = dt
        self.damping = damping
        self.mass = mass
        self.momentum_loss = momentum_loss
        self.radius = radius
        self.displacement = self.position - self.old_position
        self.__apply_inverse_mass()

    def __apply_inverse_mass(self):
        if self.mass == 0:
            self.inverse_mass = 0
        else:
            self.inverse_mass = 1/self.mass

    def apply_verlet(self):
        if self.inverse_mass == 0:
            self.old_position = self.position.copy()
            self.displacement = pg.Vector2(0, 0)
            return

        displacement = (self.position - self.old_position) * self.damping
        self.old_position = self.position.copy()
        self.displacement = displacement
        self.position = self.position + displacement + self.wind + self.gravity * self.dt ** 2

    def correct_approx_distance(self, particle_b: Particle, rest_length: float, stiffness: float = 1.0):
        delta = self.position - particle_b.position
        distance_squared = delta.length_squared()
        rest_squared = rest_length * rest_length
        denominator = distance_squared + rest_squared
        total_inverse_mass = self.inverse_mass + particle_b.inverse_mass

        if denominator == 0.0 or total_inverse_mass == 0.0:
            pass
        else:
            factor = (distance_squared - rest_squared) / (distance_squared + rest_squared)
            correction = delta * factor * stiffness
            move_a = correction * (self.inverse_mass / total_inverse_mass)
            move_b = correction * (particle_b.inverse_mass / total_inverse_mass)
            self.position = self.position - move_a
            particle_b.position = particle_b.position + move_b 

    def distance_correction_iterations(self, N: int, particle_b: Particle, rest_length: float):
        for _ in range(N):
            self.correct_approx_distance(particle_b=particle_b, rest_length=rest_length)

    def check_bounday_collision(self, width_x: int, width_y: int):
        # Collision detection for X-axis
        if self.position.x >= width_x - self.radius:
            self.position.x = width_x - self.radius
            self.old_position.x = self.position.x + self.momentum_loss * self.displacement.x
        elif self.position.x <= self.radius:
            self.position.x = self.radius
            self.old_position.x = self.position.x + self.momentum_loss * self.displacement.x

        # Collision detection for Y-axis
        if self.position.y >= width_y - self.radius:
            self.position.y = width_y - self.radius
            self.old_position.y = self.position.y + self.momentum_loss * self.displacement.y
        elif self.position.y <= self.radius:
            self.position.y = self.radius
            self.old_position.y = self.position.y + self.momentum_loss * self.displacement.y
