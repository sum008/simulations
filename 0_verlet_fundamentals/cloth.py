import pygame as pg
from random import randint

pg.init()
width_x, width_y = 900, 800
screen = pg.display.set_mode((width_x, width_y))
clock = pg.time.Clock()
running = True

# particle_pos = pg.Vector2(70, 30)
# particle_old_pos = pg.Vector2(65, 15)
gravity = pg.Vector2(0, 50)
wind = pg.Vector2(0, 0)
# vel = pg.Vector2(15,15)
dt = 1 / 60
radius = 2
e = 0.8
rest_length = 5
dis = rest_length

M = 70
N = 70
particle_pos, particle_old_pos, vel = [], [], []
def initialize_particles(particle_pos: list, particle_old_pos: list, vel: list):
    dis_y=0
    for i in range(M):
        v, pp, pop = [], [], []
        base_x, base_y, dis_x = 20, 20, 0
        for j in range(N):
            pop.append(pg.Vector2(base_x+dis_x, base_y+dis_y))
            v.append(pg.Vector2(0, 0))
            # pop.append(pg.Vector2(pp[j].x - v[j].x, pp[j].y - v[j].y))
            pp.append(pg.Vector2(base_x+dis_x, base_y+dis_y))
            dis_x+=dis
            
        dis_y+=dis
        dis_x=dis
        vel.append(v)
        particle_pos.append(pp)
        particle_old_pos.append(pop)


initialize_particles(particle_pos=particle_pos, particle_old_pos=particle_old_pos, vel=vel)
print(len(particle_pos), len(particle_pos[0]))

def apply_collision(width_x, width_y, particle_pos: list, particle_old_pos: list, vel, radius, e):
    for i in range(M):
        for j in range(N):
            if particle_pos[i][j].x >= width_x - radius:
                particle_pos[i][j].x = width_x - radius
                particle_old_pos[i][j].x = particle_pos[i][j].x + e * vel[i][j].x
            elif particle_pos[i][j].x <= radius:
                particle_pos[i][j].x = radius
                particle_old_pos[i][j].x = particle_pos[i][j].x + e * vel[i][j].x

            if particle_pos[i][j].y >= width_y - radius:
                particle_pos[i][j].y = width_y - radius
                particle_old_pos[i][j].y = particle_pos[i][j].y + e * vel[i][j].y
            elif particle_pos[i][j].y <= radius:
                particle_pos[i][j].y = radius
                particle_old_pos[i][j].y = particle_pos[i][j].y + e * vel[i][j].y

    # return particle_old_pos, particle_pos

def apply_verlet(particle_pos: list, particle_old_pos: list, gravity, vel, dt):
    for i in range(M):
        for j in range(N):
            vel[i][j].x = particle_pos[i][j].x - particle_old_pos[i][j].x
            vel[i][j].y = particle_pos[i][j].y - particle_old_pos[i][j].y

            particle_old_pos[i][j] = particle_pos[i][j].copy()
            particle_pos[i][j].x = particle_old_pos[i][j].x + vel[i][j].x + wind.x
            particle_pos[i][j].y = particle_old_pos[i][j].y + vel[i][j].y + gravity.y * dt**2
    # return particle_old_pos, particle_pos

def constraint_correction(particle_pos, a, b, c, d, constraint_lenght=rest_length, stiffness=1.0):
    delta = particle_pos[a][b] - particle_pos[c][d]
    distance = particle_pos[a][b].distance_to(particle_pos[c][d])
    if distance != 0:
        diff = (distance - constraint_lenght) / distance #total length that is changed from rest_length
        correction = delta * diff #total correction percentage
        particle_pos[a][b] -= correction * 0.5 * stiffness
        particle_pos[c][d] += correction * 0.5 * stiffness

def apply_bending_constraints(particle_pos):
    bend_length = rest_length * 2
    for i in range(M):
        for j in range(N):
            # horizontal bending
            if j + 2 < N:
                constraint_correction(particle_pos,i, j, i, j + 2, bend_length, stiffness=0.3)

            # vertical bending
            if i + 2 < M:
                constraint_correction(particle_pos, i, j, i + 2, j, bend_length, stiffness=0.3)
    

def apply_distance_constrains(particle_pos: list[pg.Vector2]):
    for i in range(M):
        for j in range(N):
            #right
            if j+1<N:
                constraint_correction(particle_pos, i, j, i, j+1, rest_length)

            #down
            if i+1<M:
                constraint_correction(particle_pos, i, j, i+1, j, rest_length)

                
mouse_pos=pg.Vector2(0,0)
while running:
    for event in pg.event.get():
        if event.type == pg.QUIT:
            running = False

    screen.fill("black")
    # Verlet integration
    apply_verlet(particle_pos, particle_old_pos, gravity, vel, dt)

    for i in range(5):
        # Apply distance constraint
        apply_distance_constrains(particle_pos)
        # apply_bending_constraints(particle_pos)

        if pg.mouse.get_pressed()[0]:
            mouse_pos = pg.Vector2(pg.mouse.get_pos())
    
            particle_pos[M-1][N-1] = mouse_pos
            particle_old_pos[M-1][N-1] = mouse_pos
    
            # particle_pos[0][N-1] = pg.Vector2(mouse_pos.x + rest_length * (N - 1), mouse_pos.y)
            # particle_old_pos[0][N-1] = pg.Vector2(mouse_pos.x + rest_length * (N - 1), mouse_pos.y)
            
        particle_pos[0][0] = pg.Vector2(100, 100)
        particle_old_pos[0][0] = particle_pos[0][0].copy()

        particle_pos[0][N-1] = pg.Vector2(
            100 + rest_length * (N - 1),
            100
        )
        particle_old_pos[0][N-1] = particle_pos[0][N-1].copy()
         

    # apply_collision(width_x, width_y, particle_pos, particle_old_pos, vel, radius, e)

    for i in range(M):
        for j in range(N):
            if j + 1 < N:
                pg.draw.line(screen, "gray", particle_pos[i][j], particle_pos[i][j + 1], 1)
            if i + 1 < M:
                pg.draw.line(screen, "gray", particle_pos[i][j], particle_pos[i + 1][j], 1)
            # pg.draw.circle(screen, "white", particle_pos[i][j], radius)

    # for i in range(M):
    #     for j in range(N):
    #         pg.draw.line(screen, "white", particle_pos[i][j], particle_pos[index+1], 1)

    pg.display.flip()
    clock.tick(60)
pg.quit()