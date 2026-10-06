from random import randint

import pygame as pg
from particle import Particle

pg.init()
WIDTH, HEIGHT = 900, 800
screen = pg.display.set_mode((WIDTH, HEIGHT))
clock = pg.time.Clock()
font = pg.font.Font(None, 24)

gravity = pg.Vector2(0, 50)
wind = pg.Vector2(0, 0)
dt = 1 / 60
REST_LENGTH = 5
ROWS = 50
COLUMNS = 50
SOLVER_ITERATIONS = 3
CONSTRAINT_STIFFNESS = 0.8
RUNNING = True

particle_pos, particle_old_pos = [], []

particles: list[list[Particle]] = []
def initialize_particles():

    for i in range(ROWS):
        particle_row: list[Particle] = []
        for j in range(COLUMNS):
            position = pg.Vector2(20 + j * REST_LENGTH, 20 + i * REST_LENGTH)
            particle = Particle(position=position, old_position=position.copy(), gravity=gravity, wind=wind, dt=dt, radius=5, mass=randint(2, 4))
            particle_row.append(particle)
        particles.append(particle_row)


def apply_distance_constraints(stiffness):
    for i in range(ROWS):
        for j in range(COLUMNS):
            if j + 1 < COLUMNS:
                particles[i][j].correct_approx_distance(particle_b=particles[i][j+1], stiffness=stiffness, rest_length=REST_LENGTH)
                # constraint_correction(i, j, i, j + 1, stiffness=stiffness)
            if i + 1 < ROWS:
                particles[i][j].correct_approx_distance(particle_b=particles[i+1][j], stiffness=stiffness, rest_length=REST_LENGTH)
                # constraint_correction(i, j, i + 1, j, stiffness=stiffness)


def pin_corners(corners):
    for row, column, position in corners:
        particle = particles[row][column]
        particle.position = position.copy()
        particle.old_position = position.copy()

                
initialize_particles()

mouse_pos = pg.Vector2(0, 0)
while RUNNING:
    for event in pg.event.get():
        if event.type == pg.QUIT:
            RUNNING = False
        elif event.type == pg.KEYDOWN:
            if event.key == pg.K_LEFTBRACKET:
                CONSTRAINT_STIFFNESS = max(0.05, CONSTRAINT_STIFFNESS - 0.05)
            elif event.key == pg.K_RIGHTBRACKET:
                CONSTRAINT_STIFFNESS = min(1.0, CONSTRAINT_STIFFNESS + 0.05)

    screen.fill("black")
    pg.display.set_caption(
        f"Verlet Cloth - stiffness: {CONSTRAINT_STIFFNESS:.2f} ([ / ])"
    )

    for i in range(ROWS):
        for j in range(COLUMNS):
            particles[i][j].apply_verlet()

    mouse_pressed = pg.mouse.get_pressed()[0]
    if mouse_pressed:
        mouse_pos = pg.Vector2(pg.mouse.get_pos())

    cloth_width = REST_LENGTH * (COLUMNS - 1)
    cloth_height = REST_LENGTH * (ROWS - 1)
    bottom_right = pg.Vector2(100 + cloth_width, 100 + cloth_height)
    if mouse_pressed:
        bottom_right = mouse_pos

    top_left = bottom_right - pg.Vector2(cloth_width, cloth_height)
    top_right = bottom_right - pg.Vector2(0, cloth_height)
    bottom_left = bottom_right - pg.Vector2(cloth_width, 0)
    corners = (
        (0, 0, top_left),
        (0, COLUMNS - 1, top_right),
        (ROWS - 1, 0, bottom_left),
        (ROWS - 1, COLUMNS - 1, bottom_right),
    )

    for _ in range(SOLVER_ITERATIONS):
        apply_distance_constraints(CONSTRAINT_STIFFNESS)
        pin_corners(corners)

    for i in range(ROWS):
        for j in range(COLUMNS):
            if j + 1 < COLUMNS:
                pg.draw.line(screen, "gray", particles[i][j].position, particles[i][j + 1].position, 1)
            if i + 1 < ROWS:
                pg.draw.line(screen, "gray", particles[i][j].position, particles[i + 1][j].position, 1)

    clock.tick(90)
    fps = clock.get_fps()
    fps_text = font.render(f"FPS: {fps:.1f}", True, "white")
    screen.blit(fps_text, (10, 10))
    pg.display.flip()

pg.quit()