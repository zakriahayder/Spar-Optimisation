import openmdao.api as om
import numpy as np


class LiftDistribution(om.ExplicitComponent):
    """Computes elliptical lift distribution along wing span."""
    
    def initialize(self):
        """Declare number of spanwise points."""
        self.options.declare('num_points', types=int, default=100)
    
    def setup(self):
        """Define inputs and outputs."""
        n = self.options['num_points']
        
        # Inputs
        self.add_input('x', val=np.zeros(n), desc='Spanwise positions [m]')
        self.add_input('b', val=11.23, desc='Wingspan [m]')
        self.add_input('L', val=0.0, desc='Total lift force [N]')
        
        # Outputs
        self.add_output('Lprime', val=np.zeros(n), desc='Lift load distribution [N/m]')
        
        # Use finite differences for derivatives
        self.declare_partials('*', '*', method='fd')
    
    def compute(self, inputs, outputs):
        """Calculate elliptical lift distribution."""
        x = inputs['x']
        b = inputs['b']
        L = inputs['L']
        
        # Compute lift per unit span using elliptical distribution
        outputs['Lprime'] = (4 * L) / (b * np.pi) * np.sqrt(1 - (2 * x / b)**2)