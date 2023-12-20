import pygame
import sys
import math
import numpy

# Initialize Pygame
pygame.init()

# Variables
width, height = 1000, 600
len1 = 180              # Length of rods
len2 = 260
mass1 = 10              # Masses
mass2 = 10
theta1 = math.pi / 2    # Angle of weights
theta2 = -math.pi / 2
theta1_v = 0            # Initial velocity
theta2_v = 0
g = 1                   # Gravity

# Anchor point
originx, originy = width // 2, 100

trailing_line = [] 

screen = pygame.display.set_mode((width, height))
pygame.display.set_caption("Double Pendulum")
clock = pygame.time.Clock()

# Calculates total energy (to see fluctuation)
def calculate_total_energy():
    K1 = 0.5 * mass1 * len1**2 * theta1_v**2
    K2 = 0.5 * mass2 * (len1**2 * theta1_v**2 + len2**2 * theta2_v**2 + 2 * len1 * len2 * theta1_v * theta2_v * math.cos(theta1 - theta2))

    y1 = -len1 * math.cos(theta1)
    y2 = y1 - len2 * math.cos(theta2)

    V = mass1 * g * y1 + mass2 * g * y2

    return K1 + K2 + V

# Defines the equations of motion (derived from Euler-Lagrange)
def eulerlagrange(t1, t2, w1, w2):
    theta1 = (len2 / len1) * (mass2 / (mass1 + mass2)) * math.cos(t1 - t2)
    theta2 = (len1 / len2) * math.cos(t1 - t2)

    f1 = -(len2 / len1) * (mass2 / (mass1 + mass2)) * (w2**2) * math.sin(t1 - t2) - (g / len1) * math.sin(t1)
    f2 = (len1 / len2) * (w1**2) * math.sin(t1 - t2) - (g / len2) * math.sin(t2)

    o1 = (f1 - theta1 * f2) / (1 - theta1 * theta2)
    o2 = (f2 - theta2 * f1) / (1 - theta1 * theta2)

    return numpy.array([w1, w2, o1, o2])

# Perform time step updating angles, solving equations using Runge-Kutta method,
# (Haven't covered differential equations yet, so this is something of a black box implementation)
def time_step(dt):
    global theta1, theta2, theta1_v, theta2_v

    y = numpy.array([theta1, theta2, theta1_v, theta2_v])

    # Computes with equations of motion
    rk1 = eulerlagrange(*y)
    rk2 = eulerlagrange(*(y + dt * rk1 / 2))
    rk3 = eulerlagrange(*(y + dt * rk2 / 2))
    rk4 = eulerlagrange(*(y + dt * rk3))

    R = 1.0 / 6.0 * dt * (rk1 + 2.0 * rk2 + 2.0 * rk3 + rk4)

    # Updates angles and angular velocity
    theta1 += R[0]
    theta2 += R[1]
    theta1_v += R[2]
    theta2_v += R[3]

# Main loop
while True:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            sys.exit()

    # Control game speed
    clock.tick(60)  # frames per second

    screen.fill((175, 175, 175))

    # Perform time step
    time_step(1/2)  # steps per frame

    # Transform coordinates back and updates with new angles
    x1 = len1 * math.sin(theta1)
    y1 = len1 * math.cos(theta1)

    x2 = x1 + len2 * math.sin(theta2)
    y2 = y1 + len2 * math.cos(theta2)


    # Draws trail with stored positions
    trailing_line.append((originx + int(x2), originy + int(y2)))
    for i in range(1, len(trailing_line)):
        pygame.draw.line(screen, (100, 100, 100), trailing_line[i - 1], trailing_line[i], 2)
    

    # Draw objects
    pygame.draw.line(screen, (0, 0, 0), (originx, originy), (originx + int(x1), originy + int(y1)), 1)
    pygame.draw.circle(screen, (0, 0, 0), (originx + int(x1), originy + int(y1)), 6)

    pygame.draw.line(screen, (0, 0, 0), (originx + int(x1), originy + int(y1)), (originx + int(x2), originy + int(y2)), 1)
    pygame.draw.circle(screen, (0, 0, 0), (originx + int(x2), originy + int(y2)), 6)


    # Total energy
    total_energy = calculate_total_energy()
    font = pygame.font.Font(None, 26)
    energy_text = font.render(f'Total Energy: {total_energy:.2f}', True, (0, 0, 0))
    screen.blit(energy_text, (10, 10))

    pygame.display.flip()
