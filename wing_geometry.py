import openmdao.api as om
import numpy as np


class WingGeometry(om.ExplicitComponent):
    """Computes geometry for elliptical wing given wingspan and area."""
    
    def initialize(self):
        self.options.declare('num_points', types=int, default=100, desc='Number of spanwise discretization points')
    
    def setup(self):
        """Sets up inputs/outputs."""
        n = self.options['num_points']
        
        # Inputs
        self.add_input('b', val=11.23, desc='Wingspan [m]')
        self.add_input('S', val=22.48, desc='Wing area [m^2]')
        
        # Outputs
        self.add_output('x', val=np.zeros(n), desc='Spanwise positions [m]')
        self.add_output('cChord', val=np.zeros(n), desc='Chord length at each spanwise position [m]')
        self.add_output('cRoot', val=0.0, desc='Root chord length [m]')
        
        self.declare_partials('*', '*', method='fd')
    
    def compute(self, inputs, outputs):
        """Calculates wing geometry."""
        n = self.options['num_points']
        b = inputs['b']
        S = inputs['S']
        
        x = np.linspace(0, b/2, n)  # Spanwise coords
        outputs['x'] = x
        
        cRoot = (4 * S) / (np.pi * b)  # Root chord
        outputs['cRoot'] = cRoot
        
        outputs['cChord'] = cRoot * np.sqrt(1 - (2 * x / b)**2)  # Chord distribution