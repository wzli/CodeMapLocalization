#!/usr/bin/env python3

import matplotlib.pyplot as plt
import numpy as np

N = 64
indices = np.arange(N)
r = (indices - 31.5) * np.pi / 44
lookup_table = np.cos(r)**2

lookup_table[:10] = 0
lookup_table[54:] = 0

print('{', end='')
for i, val in enumerate(lookup_table):
    if i % 4 == 0:
        print("\n    ", end='')
    print(f"{round(val, 8)}f, ", end='')
print('\n}')

plt.plot(indices, lookup_table)
plt.show()
