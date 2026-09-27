import pygame as pg
from random import randint

pg.init()
width_x, width_y = 800, 600
screen = pg.display.set_mode((width_x, width_y))
clock = pg.time.Clock()
running = True

# particle_pos = pg.Vector2(70, 30)
# particle_old_pos = pg.Vector2(65, 15)
gravity = pg.Vector2(0, 550)
# vel = pg.Vector2(15,15)
dt = 1 / 60
radius = 15
e = 0.8

N = 3
particle_pos, particle_old_pos, vel = [], [], []
def initialize_particles(N: int, particle_pos: list, particle_old_pos: list, vel: list):
    for index in range(N):
        vel.append(pg.Vector2(randint(5, 20), randint(10, 20)))
        particle_pos.append(pg.Vector2(randint(30, 600), randint(30, 400)))
        particle_old_pos.append(pg.Vector2(particle_pos[index].x - vel[index].x, particle_pos[index].y - vel[index].y))

initialize_particles(N, particle_pos=particle_pos, particle_old_pos=particle_old_pos, vel=vel)


def apply_collision(width_x, width_y, particle_pos: list, particle_old_pos: list, vel, radius, e):
    for index in range(N):
        if particle_pos[index].x >= width_x - radius:
            particle_pos[index].x = width_x - radius
            particle_old_pos[index].x = particle_pos[index].x + e*vel[index].x
        elif particle_pos[index].x <= radius:
            particle_pos[index].x = radius
            particle_old_pos[index].x = particle_pos[index].x + e*vel[index].x

        # Collision detection of Y-axis
        if particle_pos[index].y >= width_y - radius:
            particle_pos[index].y = width_y - radius
            particle_old_pos[index].y = particle_pos[index].y + e*vel[index].y
        elif particle_pos[index].y <= radius:
            particle_pos[index].y = radius
            particle_old_pos[index].y = particle_pos[index].y + e*vel[index].y

    return particle_old_pos, particle_pos

def apply_verlet(particle_pos: list, particle_old_pos: list, gravity, vel, dt):
    for index in range(N):
        vel[index].x = particle_pos[index].x - particle_old_pos[index].x
        vel[index].y = particle_pos[index].y - particle_old_pos[index].y
        particle_old_pos[index] = particle_pos[index].copy()
        particle_pos[index].x = particle_old_pos[index].x + vel[index].x
        particle_pos[index].y = particle_old_pos[index].y + vel[index].y + gravity.y * dt**2
    return particle_old_pos, particle_pos

while running:
    for event in pg.event.get():
        if event.type == pg.QUIT:
            running = False
    screen.fill("black")


    # Verlet integration
    particle_old_pos, particle_pos = apply_verlet(particle_pos, particle_old_pos, gravity, vel, dt)

    # Collition detection for X-axis
    particle_old_pos, particle_pos = apply_collision(width_x, width_y, particle_pos, particle_old_pos, vel, radius, e)

    for index in range(N):
        pg.draw.circle(screen, "white", particle_pos[index], radius)

    pg.display.flip()
    clock.tick(90)
pg.quit()