import pygame as pg

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
SOLVER_ITERATIONS = 9
CONSTRAINT_STIFFNESS = 0.8
RUNNING = True

particle_pos, particle_old_pos = [], []


def initialize_particles():
    for i in range(ROWS):
        positions, old_positions = [], []
        for j in range(COLUMNS):
            position = pg.Vector2(20 + j * REST_LENGTH, 20 + i * REST_LENGTH)
            positions.append(position)
            old_positions.append(position.copy())
        particle_pos.append(positions)
        particle_old_pos.append(old_positions)

def apply_collision(width_x, width_y, particle_pos: list, particle_old_pos: list, vel, radius, e):
    for i in range(ROWS):
        for j in range(COLUMNS):
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


def apply_verlet():
    for i in range(ROWS):
        for j in range(COLUMNS):
            current_position = particle_pos[i][j]
            velocity = current_position - particle_old_pos[i][j]
            particle_old_pos[i][j] = current_position.copy()
            particle_pos[i][j] = (
                current_position + velocity + wind + gravity * dt**2
            )


def constraint_correction(
    a, b, c, d, constraint_length=REST_LENGTH, stiffness=1.0
):
    delta = particle_pos[a][b] - particle_pos[c][d]
    distance = particle_pos[a][b].distance_to(particle_pos[c][d])
    if distance != 0:
        diff = (distance - constraint_length) / distance
        correction = delta * diff * stiffness
        particle_pos[a][b] -= correction * 0.5
        particle_pos[c][d] += correction * 0.5


def apply_distance_constraints(stiffness):
    for i in range(ROWS):
        for j in range(COLUMNS):
            if j + 1 < COLUMNS:
                constraint_correction(i, j, i, j + 1, stiffness=stiffness)
            if i + 1 < ROWS:
                constraint_correction(i, j, i + 1, j, stiffness=stiffness)


def pin_corners(corners):
    for row, column, position in corners:
        particle_pos[row][column] = position
        particle_old_pos[row][column] = position.copy()

                
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
    apply_verlet()

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
                pg.draw.line(screen, "gray", particle_pos[i][j], particle_pos[i][j + 1], 1)
            if i + 1 < ROWS:
                pg.draw.line(screen, "gray", particle_pos[i][j], particle_pos[i + 1][j], 1)

    clock.tick(90)
    fps = clock.get_fps()
    fps_text = font.render(f"FPS: {fps:.1f}", True, "white")
    screen.blit(fps_text, (10, 10))
    pg.display.flip()

pg.quit()