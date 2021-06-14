#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Jun 13 16:09:21 2021

@author: algutier
"""

from dataclasses import dataclass, field
from typing import Any
from heapq import *
from configuration import *

"""
An object to insert in a heap. Do not compare the item only the value attribute
"""
"""

ORIGINAL

"""
""" 
@dataclass(order=True)
class Heap_item:
    value: float
    item: Any=field(compare=False)
"""
@dataclass(order=True)
class Heap_item:
    value: (float, float, Configuration)
    item: Any=field(compare=False)
   
if __name__ == "__main__":
    item_1 = Heap_item(15, (9, 6, [1, 4, 3]))
    item_2 = Heap_item(10, (4, 6, [1, 3, 0]))
    item_3 = Heap_item(13, (7, 6, [1, 4, 2]))
    heap = []
    heappush(heap, item_1)
    heappush(heap, item_2)
    heappush(heap, item_3)
    print(heappop(heap))
    print(heappop(heap))
    (g_cost, h_cost, config) = heappop(heap).item
    print(config)
    print(g_cost)
    print(h_cost)
    