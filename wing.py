import openmdao.api as om
import numpy as np


class WingGeometry(om.ExplicitComponent):

    def initialize(self):
        self.options.declare(
            "num_points",
            types=int,
            default=100,
            desc="Number of spanwise discretization points",
        )

    def setup(self):
        n = self.options["num_points"]

        self.add_input("b", val=11.23, desc="Wingspan [m]")
        self.add_input("S", val=22.48, desc="Wing area [m^2]")

        self.add_output("x", val=np.zeros(n), desc="Spanwise positions [m]")
        self.add_output(
            "c_chord", val=np.zeros(n), desc="Chord lenght at each spanwise postion [m]"
        )
        self.add_output("c_root", val=0.0, desc="Root chord lenght [m]")

        self.declare_partials("*", "*", method="fd")

    def compute(self, inputs, outputs):
        n = self.options["num_points"]

        b = inputs["b"]
        S = inputs["S"]

        c_root = (4 * S) / (np.pi * b)
        outputs["c_root"] = c_root

        x = np.linspace(0, b / 2, n)
        outputs["x"] = x

        c_chord = c_root * np.sqrt(1 - (2 * x / b) ** 2)
        outputs["c_chord"] = c_chord
