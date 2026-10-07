"""Keep every exported DTHC6 HAL name within LinuxCNC's name limit."""

import pathlib
import re
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[2]
COMP = ROOT / "src/hal/drivers/ccc_dthc_bspi.comp"
HAL_H = ROOT / "src/hal/hal.h"


class DthcBspiNamesTest(unittest.TestCase):
    def test_exported_names_fit_hal_limit(self):
        limit = int(re.search(r"#define\s+HAL_NAME_LEN\s+(\d+)",
                              HAL_H.read_text()).group(1))
        source = COMP.read_text()
        component = re.search(r"^component\s+(\S+)", source, re.M).group(1)
        prefix = component.replace("_", "-") + "."
        declarations = re.findall(
            r"^(?:pin\s+(?:in|out|io)\s+|param\s+(?:rw|ro)\s+)"
            r"(?:bit|float|s32|u32)\s+([^\s;=]+)", source, re.M)
        self.assertGreater(len(declarations), 100)
        for name in declarations:
            with self.subTest(name=name):
                self.assertLessEqual(len(prefix + name), limit)


if __name__ == "__main__":
    unittest.main()
