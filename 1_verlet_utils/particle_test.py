import pygame as pg
from random import randint
from particle import Particle

pg.init()
width_x, width_y = 800, 600
screen = pg.display.set_mode((width_x, width_y))
lock = pg.time.Clock()
font = pg.font.Font(None, 24)
clock = pg.time.Clock()
running = True

gravity = pg.Vector2(0, 550)
wind = pg.Vector2(0,0)

N = 30
particles: list[Particle] = []
def initialize_particles(N: int):
    for index in range(N):
        position=pg.Vector2(randint(30, 600), randint(30, 400))
        displacement=pg.Vector2(randint(5, 20), randint(10, 20))
        old_position=pg.Vector2(position.x - displacement.x, position.y - displacement.y)
        particle: Particle = Particle(position=position, old_position=old_position, gravity=gravity,
                            dt=1/60, radius=5, damping=1, momentum_loss=0.9, wind=wind, mass=randint(2, 10))
        particles.append(particle)

initialize_particles(N)

while running:
    for event in pg.event.get():
        if event.type == pg.QUIT:
            running = False
    screen.fill("black")

    for particle in particles:
        particle.apply_verlet()
        particle.check_bounday_collision(width_x=width_x, width_y=width_y)

    for particle in particles:
        pg.draw.circle(screen, "white", particle.position, particle.radius)

    clock.tick(90)
    fps = clock.get_fps()
    fps_text = font.render(f"FPS: {fps:.1f}", True, "white")
    screen.blit(fps_text, (10, 10))
    pg.display.flip()
    
pg.quit()