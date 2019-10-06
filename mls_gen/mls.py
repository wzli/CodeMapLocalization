class LFSR:
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

    def __init__(self, n_bits, seed=1):
        if n_bits not in self.taps:
            raise AssertionError(
                f"LFSR polynomial for n_bits = {n_bits} is not in table")
        self.n_bits = n_bits
        self.reset(seed)

    def reset(self, seed=1):
        if seed <= 0:
            raise AssertionError(f"LFSR seed must be non-zero")
        self.state = seed
        self.steps = 0
        self.cycles = 0

    def next(self):
        bit = 0
        for tap in self.taps[self.n_bits]:
            bit = bit ^ (self.state >> (self.n_bits - tap))
        self.state = (self.state >> 1) | ((bit & 1) << (self.n_bits - 1))
        self.steps += 1
        if self.steps > (1 << self.n_bits) - 2:
            self.steps = 0
            self.cycles += 1
        return self.state, bit & 1

    def sequence(self, seed=None):
        self.reset(self.state if not seed else seed)
        while self.cycles == 0:
            yield self.next()

    def sequence_string(self):
        return ''.join(str(bit) for _, bit in self.sequence())
