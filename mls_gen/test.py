#!/usr/bin/env python3

import numpy as np
from matplotlib import pyplot as plt


def reverseBits(num, bits):
    return sum(1 << (bits - 1 - i) for i in range(bits) if num >> i & 1)


taps = {
    2: [2, 1],
    3: [3, 2],
    4: [4, 3],
    5: [5, 3],
    6: [6, 5],
    7: [7, 6],
    8: [8, 6, 5, 4],
    9: [9, 5],
    10: [10, 7],
    11: [11, 9],
    12: [12, 11, 10, 4],
    13: [13, 12, 11, 8],
    14: [14, 13, 12, 2],
    15: [15, 14],
    16: [16, 15, 13, 4],
    17: [17, 14],
    18: [18, 11],
    19: [19, 18, 17, 14],
    20: [20, 17],
    21: [21, 19],
    22: [22, 21],
    23: [23, 18],
    24: [24, 23, 22, 17],
}


def lfsr(x, n):
    bit = 0
    for tap in taps[n]:
        bit = bit ^ (x >> (n - tap))
    last_x = x
    x = (x >> 1) | ((bit & 1) << (n - 1))
    return x


n = 7
seed = 1
non_rev = set()

N = 2**n - 1

S = np.empty(N, dtype=np.uint16)
LUT = np.empty(N + 1, dtype=np.uint16)

seq = []
for i in range(N):
    S[i] = lfsr(seed, n)
    LUT[S[i]] = seed & 1 | i << 1
    S[i] = S[i] << 1 | (seed & 1)
    seed = S[i] >> 1
    seq.append(S[i] & 1)

for i in range(N + 1):
    print(f'{i:0{n}b} {LUT[i] & 1} {LUT[i] >> 1} {reverseBits(i, n):0{n}b}')

for i in range(N - 2):
    print(f'{S[i] >> 1:0{n}b} {i}')
    ai = S[i] >> 1
    rai = reverseBits(ai, n)
    bi = S[i + 1] >> 1
    rbi = reverseBits(bi, n)
    ci = S[i + 2] >> 1
    rci = reverseBits(ci, n)

    a = LUT[ai] >> 1
    b = LUT[bi] >> 1
    c = LUT[ci] >> 1

    ra = LUT[rai] >> 1
    rb = LUT[rbi] >> 1
    rc = LUT[rci] >> 1

    #if (rc + 1 == rb and rb + 1 == ra):
    #print(f'    {ai:0{n}b} {a} {bi:0{n}b} {b} {ci:0{n}b} {c}    {rci:0{n}b} {rc} {rbi:0{n}b} {rb} {rai:0{n}b} {ra}')
    #print(f'  invaid {rc + 1 == rb} {rb + 1 == ra}')

print(f'{n}, {N}, {np.array(list(reversed(seq)))})')

pool = {}
for i in range(N):
    key = S[i].tobytes() if (LUT[S[i] >> 1]
                             & 1) == 0 else np.invert(S[i]).tobytes()
    if key in pool:
        print(f'collide with {pool[key]}')
    pool[key] = i

print(f'LUT {LUT.nbytes} Bytes')

#seq = np.array(seq) * 2 - 1
#M = np.outer(seq, seq)
#plt.imshow(M, interpolation='nearest')
#plt.show()

k = 6
rev = (
    0,
    0,
    0,
    0,
    0,
    1,
    0,
    0,
    1,
    0,
    1,
    0,
    0,
    0,
    1,
    1,
    0,
    0,
    1,
    1,
    1,
    0,
    1,
    0,
    1,
    1,
    0,
    1,
    1,
    1,
    1,
    1,
    0,
    0,
    0,
    0,
    0,
    1,
)
rev_count = 0
for i in range(len(rev) - k):
    key = rev[i:i + k]
    print(key, i)
    if key in pool:
        print(f'collide with {pool[key]}')
        rev_count = rev_count + 1
    elif tuple(reversed(key)) in pool:
        print(f'rev collide with {pool[tuple(reversed(key))]}')
        rev_count = rev_count + 1
    pool[key] = i
print(f'{rev_count}')

exit()

k = 15
pool = {}
rev_count = 0
for i in range(2**k):
    print(f'key {i:0{k}b} {i}')
    if i in poel:
        print(f'collide with {pool[i]}')
        rev_count = rev_count + 1
    elif reverseBits(i, k) in pool:
        print(f'collide with {pool[reverseBits(i,k)]}')
        rev_count = rev_count + 1
    pool[i] = i
print(f'{2**k} - {rev_count} = {2**k - rev_count}')

seed = 12101
x = seed
count = 0
while True:
    x = lfsr(x, k, True)
    count = count + 1
    if x == seed:
        break
for item in sorted(non_rev):
    print(f'{item:0{k}b}, {item}')

print(f'count {len(non_rev)}')
