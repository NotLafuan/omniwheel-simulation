import pygame
from sys import exit
from utils import *
import numpy as np
from math import cos, sin, radians, pi

WIDTH = 1280
HEIGHT = 720

pygame.init()
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption('Simulator')
clock = pygame.time.Clock()

# font = pygame.font.Font('fonts/Pixeltype.ttf', 50)

# test_surface = pygame.Surface((100, 100))
# test_surface.fill('red')

# score_surf = font.render('test', False, 'black')
# score_rect = score_surf.get_rect(center=(400, 50))


def kinematics(theta: float, vx: float, vy: float, theta_dot: float):
    r = 1
    R = 1
    theta = radians(theta)
    array = [[-sin(theta+(1*pi/4)), cos(theta+(1*pi/4)), 1/2*R],
             [-sin(theta+(3*pi/4)), cos(theta+(3*pi/4)), 1/2*R],
             [-sin(theta+(5*pi/4)), cos(theta+(5*pi/4)), 1/2*R],
             [-sin(theta+(7*pi/4)), cos(theta+(7*pi/4)), 1/2*R]]
    array = np.array(array)
    return (1/r)*np.matmul(array, [vx, vy, theta_dot])


pos_error_prev = pygame.Vector2(0, 0)
PID_i = pygame.Vector2(0, 0)
PID_i_angle = 0
angle_error_prev = 0
time_prev = time.time()
PID_total = pygame.Vector2(0, 0)
PID_angle = 0


def PID_control(current_pos: pygame.Vector2, target_pos: pygame.Vector2, current_angle: float, target_angle: float):
    global pos_error_prev
    global PID_i
    global Time
    global angle_error_prev
    global PID_i_angle
    global time_prev
    global PID_total
    global PID_angle
    if (time_diff := time.time() - time_prev) < 0.01:
        return kinematics(current_angle, PID_total.x, PID_total.y, PID_angle)
    kp = 5
    ki = 0
    kd = 2
    pos_error = target_pos - current_pos
    PID_p = kp * pos_error
    PID_i = PID_i + pygame.Vector2(ki*pos_error.x, ki*pos_error.y)
    if time_diff := time.time() - time_prev:
        PID_d = kd*((pos_error - pos_error_prev)/time_diff)
    else:
        PID_d = pygame.Vector2(0, 0)
    PID_total = PID_p + PID_i + PID_d
    print(current_pos, pos_error)
    pos_error_prev = pos_error

    kp = 1000
    ki = 0.01
    kd = 500
    
    angle_error = target_angle - current_angle
    PID_p_angle = kp * angle_error
    PID_i_angle = PID_i_angle + ki * angle_error
    if time_diff:
        PID_d_angle = kd*((angle_error - angle_error_prev)/time_diff)
    else:
        PID_d_angle = 0
    PID_angle = PID_p_angle + PID_i_angle + PID_d_angle
    angle_error_prev = angle_error

    time_prev = time.time()

    return kinematics(current_angle, PID_total.x, PID_total.y, PID_angle)


base = Robot(Time, pygame.Vector2(100, 100), pygame.Color(0, 0, 255))
base.transform = pygame.Vector2(WIDTH//2, HEIGHT//2)
base.angle = 0
start_time = time.time()
while True:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            exit()
        # if event.type == pygame.KEYDOWN:
        #     if event.key == pygame.K_w:
        #         base.move_motor(*(kinematics(base.angle, 0, -100, 0)))
        #     if event.key == pygame.K_s:
        #         base.move_motor(*(kinematics(base.angle, 0, 100, 0)))
        #     if event.key == pygame.K_a:
        #         base.move_motor(*(kinematics(base.angle, -100, 0, 0)))
        #     if event.key == pygame.K_d:
        #         base.move_motor(*(kinematics(base.angle, 100, 0, 0)))
        #     if event.key == pygame.K_q:
        #         base.move_motor(*(kinematics(base.angle, 0, 0, 100)))
        #     if event.key == pygame.K_e:
        #         base.move_motor(*(kinematics(base.angle, 0, 0, -100)))
        # if event.type == pygame.KEYUP:
        #     if event.key == pygame.K_w:
        #         base.move_motor(*(kinematics(base.angle, 0, 0, 0)))
        #     if event.key == pygame.K_s:
        #         base.move_motor(*(kinematics(base.angle, 0, 0, 0)))
        #     if event.key == pygame.K_a:
        #         base.move_motor(*(kinematics(base.angle, 0, 0, 0)))
        #     if event.key == pygame.K_d:
        #         base.move_motor(*(kinematics(base.angle, 0, 0, 0)))
        #     if event.key == pygame.K_q:
        #         base.move_motor(*(kinematics(base.angle, 0, 0, 0)))
        #     if event.key == pygame.K_e:
        #         base.move_motor(*(kinematics(base.angle, 0, 0, 0)))
        if event.type == pygame.MOUSEBUTTONUP:
            pos = pygame.mouse.get_pos()
            target_pos = pygame.Vector2(*pos)

    Time.update()
    base.update()

    screen.fill('white')
    base.blit(screen)

    # PID
    if time.time() - start_time < 0.5:
        target_pos = pygame.Vector2(250, 250)
    elif time.time() - start_time < 1:
        target_pos = pygame.Vector2(700, 500)
    target_angle = 0
    surface = pygame.Surface((10, 10))
    pygame.draw.line(screen, (255, 0, 00), target_pos, base.transform)
    base.move_motor(*(PID_control(base.transform,
                                target_pos,
                                radians(base.angle),
                                radians(target_angle))))

    pygame.display.update()
