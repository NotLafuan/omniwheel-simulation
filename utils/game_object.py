import pygame
from dataclasses import dataclass
from ._time import _Time
import numpy as np
from math import cos, sin, radians, pi

BLACK = (0, 0, 0)


@dataclass
class Robot:
    Time: _Time
    size: pygame.Vector2
    color: pygame.Color
    transform: pygame.Vector2 = pygame.Vector2(0, 0)
    angle: float = 0
    velocity: pygame.Vector2 = pygame.Vector2(0, 0)
    set_velocity: pygame.Vector2 = pygame.Vector2(0, 0)
    angular_velocity: float = 0
    set_angular_velocity: float = 0
    rect: pygame.Rect = pygame.Rect(0, 0, 0, 0)
    r: float = 1
    R: float = 1

    @property
    def theta(self):
        return radians(self.angle)

    @property
    def surface(self) -> pygame.Surface:
        surface = pygame.Surface(self.size)
        surface.set_colorkey(BLACK)
        surface.fill(self.color)
        rot = self.angle + 45
        surface = pygame.transform.rotate(surface, rot)
        self.rect = surface.get_rect()
        return surface

    def blit(self, screen: pygame.Surface):
        screen.blit(self.surface, self.transform - self.rect.center)

    def update(self):
        multiplier = 2 * self.Time.delta_time
        multiplier = pygame.math.clamp(multiplier, 0, 1)
        self.velocity = self.velocity.lerp(self.set_velocity, multiplier)
        self.transform += self.velocity * self.Time.delta_time
        self.angular_velocity = pygame.math.lerp(
            self.angular_velocity, self.set_angular_velocity, multiplier)
        self.angle += self.angular_velocity * self.Time.delta_time

    def move_motor(self, w1: float, w2: float, w3: float, w4: float):
        array = [[-sin(self.theta+(1*pi/4)), cos(self.theta+(1*pi/4)), 1/2*self.R],
                 [-sin(self.theta+(3*pi/4)), cos(self.theta+(3*pi/4)), 1/2*self.R],
                 [-sin(self.theta+(5*pi/4)), cos(self.theta+(5*pi/4)), 1/2*self.R],
                 [-sin(self.theta+(7*pi/4)), cos(self.theta+(7*pi/4)), 1/2*self.R]]
        array = np.array(array)
        speed = np.array([[w1*100],
                          [w2*1.0],
                          [w3*1.02],
                          [w4*1.05]])
        result = (self.r/2) * np.matmul(array.transpose(), speed)
        self.set_velocity.x, self.set_velocity.y, self.set_angular_velocity =\
            result.transpose()[0]
