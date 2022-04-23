#! /usr/bin/env python


from p5 import Vector

class Boid():

    def __init__(self, x, y) -> None:
        self.position = Vector(x, y, z=0)

