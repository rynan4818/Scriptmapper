from math import degrees, atan2, sqrt

class CompatRandom:
    """
    CPython 3.x の random.seed(n) のロジックを完全再現したクラス。
    整数や小数のシード値を受け取り、本家と全く同じ乱数初期状態を生成します。
    """
    def __init__(self, seed=0):
        self.mt = [0] * 624
        self.index = 624 + 1
        self.seed(seed)

    def init_genrand(self, s):
        self.mt[0] = s & 0xffffffff
        for i in range(1, 624):
            self.mt[i] = (1812433253 * (self.mt[i-1] ^ (self.mt[i-1] >> 30)) + i) & 0xffffffff
        self.index = 624

    def init_by_array(self, init_key):
        self.init_genrand(19650218)
        i = 1
        j = 0
        k = 624 if 624 > len(init_key) else len(init_key)
        for _ in range(k):
            self.mt[i] = (self.mt[i] ^ ((self.mt[i-1] ^ (self.mt[i-1] >> 30)) * 1664525)) + init_key[j] + j
            self.mt[i] &= 0xffffffff
            i += 1
            j += 1
            if i >= 624:
                self.mt[0] = self.mt[623]
                i = 1
            if j >= len(init_key):
                j = 0
        
        for _ in range(624 - 1):
            self.mt[i] = (self.mt[i] ^ ((self.mt[i-1] ^ (self.mt[i-1] >> 30)) * 1566083941)) - i
            self.mt[i] &= 0xffffffff
            i += 1
            if i >= 624:
                self.mt[0] = self.mt[623]
                i = 1
        
        self.mt[0] = 0x80000000

    def seed(self, a=None):
        if a is None:
            a = 0
        
        # BeatSaberマップのパラメータはfloatで来ることがあるためintに変換
        # (例: get_paramが "123.0" を返す場合など)
        if isinstance(a, float):
            a = int(a)
            
        if isinstance(a, int):
            a = abs(a)
            # 整数を32bitごとの配列に分解 (CPythonのロジック)
            if a == 0:
                keys = [0]
            else:
                keys = []
                temp_a = a
                while temp_a > 0:
                    keys.append(temp_a & 0xffffffff)
                    temp_a >>= 32
            self.init_by_array(keys)
        else:
            # 万が一文字列などが来た場合のフォールバック（今回はint/float想定）
            pass

    def extract_number(self):
        if self.index >= 624:
            self.twist()

        y = self.mt[self.index]
        y = y ^ (y >> 11)
        y = y ^ ((y << 7) & 0x9D2C5680)
        y = y ^ ((y << 15) & 0xEFC60000)
        y = y ^ (y >> 18)

        self.index += 1
        return y & 0xFFFFFFFF

    def twist(self):
        for i in range(624):
            y = (self.mt[i] & 0x80000000) + (self.mt[(i + 1) % 624] & 0x7FFFFFFF)
            self.mt[i] = (self.mt[(i + 397) % 624] ^ (y >> 1))
            if y % 2 != 0:
                self.mt[i] ^= 0x9908B0DF
        self.index = 0

    def random(self):
        # CPythonと同じ53bit精度のfloat生成
        a = self.extract_number() >> 5
        b = self.extract_number() >> 6
        return (a * 67108864.0 + b) / 9007199254740992.0
