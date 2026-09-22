import sys
import unittest
from pathlib import Path
from types import SimpleNamespace
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from BasicElements import Line, Transform, Pos, Rot
from LongCommandsUtils import rvib
from shake_data import SHAKE_LIST

class RvibParameters(unittest.TestCase):
    def generate(self, command):
        logs = []
        mapper = SimpleNamespace(logger=SimpleNamespace(log=logs.append), lines=[], lastTransform=Transform())
        line = Line(1.37)
        line.start = Transform(Pos(0, 1, -3), Rot(5, 10, 0), 60)
        line.end = Transform(Pos(2, 3, -1), Rot(10, 40, 5), 85)
        line.visibleDict = {}
        rvib(mapper, 1.37, command, line)
        rows = [[x.duration, *x.start.pos.unpack(), *x.start.rot.unpack(), x.start.fov,
                 *x.end.pos.unpack(), *x.end.rot.unpack(), x.end.fov] for x in mapper.lines]
        return rows, [s for s in logs if "!" in s]

    def test_presets_and_numeric_exponents(self):
        for preset in SHAKE_LIST:
            prefix = "rvib_" + preset
            for short, explicit in [("", ",1,1,1,1"), (",1e-2", ",0.01"),
                                    (",1,2e-1", ",1,0.2"), (",1,1,1e-1", ",1,1,0.1"),
                                    (",1,1,1,1e-1", ",1,1,1,0.1")]:
                with self.subTest(preset=preset, suffix=short):
                    actual, warnings = self.generate(prefix + short)
                    expected, _ = self.generate(prefix + explicit)
                    self.assertFalse(warnings)
                    self.assertEqual(actual, expected)
            for ease in ["ISine", "IOBack", "Drift_8_2"]:
                rows, warnings = self.generate(prefix + "," + ease)
                self.assertTrue(rows)
                self.assertFalse(warnings)

    def test_missing_and_unknown_presets(self):
        for command in ["rvib", "rvib_UNKNOWN"]:
            _, warnings = self.generate(command)
            self.assertTrue(warnings)

if __name__ == "__main__":
    unittest.main()
