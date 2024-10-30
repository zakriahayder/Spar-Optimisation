import openmdao.api as om
import numpy as np


class WingGeometry(om.ExplicitComponent):
    """Component that computes the wing geometry parameters.
    
    Calculates spanwise positions, chord lengths, and root chord based on wing area and span.
    Uses elliptical chord distribution for optimal induced drag.
    """

    def initialize(self):
        # Declare number of points for spanwise discretization
        self.options.declare(
            "num_points",
            types=int,
            default=100,
            desc="Number of spanwise discretization points",
        )

    def setup(self):
        n = self.options["num_points"]

        # Add wing geometry inputs
        self.add_input("b", val=11.23, desc="Wingspan [m]")
        self.add_input("S", val=22.48, desc="Wing area [m^2]")

        # Add computed geometry outputs
        self.add_output("x", val=np.zeros(n), desc="Spanwise positions [m]")
        self.add_output(
            "c_chord", val=np.zeros(n), desc="Chord lenght at each spanwise postion [m]"
        )
        self.add_output("c_root", val=0.0, desc="Root chord lenght [m]")

        # Use finite differences for all partial derivatives
        self.declare_partials("*", "*", method="fd")

    def compute(self, inputs, outputs):
        """Compute wing geometry parameters.
        
        Calculates:
        - Root chord length using wing area and span
        - Spanwise positions from root to tip
        - Chord distribution following elliptical planform
        """
        n = self.options["num_points"]

        b = inputs["b"]  # wingspan
        S = inputs["S"]  # wing area

        # Calculate root chord using wing area and span
        c_root = (4 * S) / (np.pi * b)
        outputs["c_root"] = c_root

        # Generate spanwise positions from root to tip
        x = np.linspace(0, b / 2, n)
        outputs["x"] = x

        # Calculate elliptical chord distribution
        c_chord = c_root * np.sqrt(1 - (2 * x / b) ** 2)
        outputs["c_chord"] = c_chord
