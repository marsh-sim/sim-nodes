class PCG:
    """
    Minimal implementation of random number generator to ensure reproducibility
    across different versions and programming languages.

    Based on minimal C implementation licensed under Apache License 2.0 
    https://www.pcg-random.org/download.html#id1
    """
    def __init__(self, seed: int):
        # state is uint64_t
        assert seed >= 0
        assert seed < 2**64
        initstate = seed
        initseq = 0

        # recreate pcg32_srandom_r below, keeping bitshifts inside uint64_t
        self.state = 0
        self.inc = ((initseq << 1) & 0xFFFF_FFFF_FFFF_FFFF) | 1  #  Controls which RNG sequence (stream) is selected. Must *always* be odd.
        _ = self._pcg32_random_r()
        self.state = (self.state + initstate) & 0xFFFF_FFFF_FFFF_FFFF
        _ = self._pcg32_random_r()

    def _pcg32_random_r(self) -> int:
        """
        Generate next random integer between 0 and 2**32-1 inclusive
        """
        oldstate = self.state
        # Advance internal state
        self.state = oldstate * 6364136223846793005 + (self.inc | 1);
        # Calculate output function (XSH RR), uses old state for max ILP
        xorshifted = ((oldstate >> 18) ^ oldstate) >> 27;
        rot = oldstate >> 59;
        return (xorshifted >> rot) | ((xorshifted << ((-rot) & 31)) & 0xFFFF_FFFF)        
    
    def randint(self, a: int, b: int) -> int:
        """
        Return a random integer N such that a <= N <= b.
        """
        # Based on pcg32_boundedrand_r
        if a == b:
            return a
        if b < a:
            a, b = b, a

        offset = a
        bound = b - a + 1  # b is inclusive
        treshold = 0x1_0000_0000 % bound
        while True:
            result = self._pcg32_random_r()
            if result >= treshold:
                return offset + (result % bound)
            

    def uniform(self, a: float, b: float) -> float:
        """
        Return a random floating-point number N such that a <= N <= b for a <= b and b <= N <= a for b < a.
        """
        fraction = self._pcg32_random_r() / float(0xFFFF_FFFF)
        return a + (b - a) * fraction
