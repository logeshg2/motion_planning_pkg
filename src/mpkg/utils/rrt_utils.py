#!/usr/bin/env python3

import numpy as np


class Node:
    def __init__(self, q, parent=None, cost=0):
        self.q = q
        self.parent = parent
        self.cost = cost
