import openmdao.api as om
import numpy as np

class SparGeometry(om.ExplicitComponent):
    """Computes geometry and section properties of a wing spar."""
    
    def initialize(self):
        self.options.declare('num_points', types=int, default=100)
    
    def setup(self):
        """Sets up inputs/outputs for spar geometry calculations."""
        n = self.options['num_points']
        
        # Inputs
        self.add_input('x', val=np.zeros(n), desc='Spanwise positions [m]')
        self.add_input('b', val=11.23, desc='Wingspan [m]')
        self.add_input('D_root', val=0.25, desc='Outer dimension at root [m]')
        self.add_input('D_tip', val=0.15, desc='Outer dimension at tip [m]')
        self.add_input('t_root', val=0.02, desc='Wall thickness at root [m]')
        self.add_input('t_tip', val=0.01, desc='Wall thickness at tip [m]')
        self.add_input('H', val=0.13, desc='Fraction of chord for spar location')
        self.add_input('cChord', val=np.zeros(n), desc='Chord length at each spanwise position [m]')
        
        # Outputs
        self.add_output('D', val=np.zeros(n), desc='Outer dimension along the span [m]')
        self.add_output('t', val=np.zeros(n), desc='Wall thickness along the span [m]')
        self.add_output('d', val=np.zeros(n), desc='Inner dimension along the span [m]')
        self.add_output('I', val=np.zeros(n), desc='Moment of inertia along the span [m^4]')
        
        self.declare_partials('*', '*', method='fd')
    
    def compute(self, inputs, outputs):
        """Calculates spar geometry and section properties."""
        x = inputs['x']
        n = len(x)
        b = inputs['b']
        D_root = inputs['D_root']
        D_tip = inputs['D_tip']
        t_root = inputs['t_root']
        t_tip = inputs['t_tip']
        H = inputs['H']
        cChord = inputs['cChord']
        
        D = D_root + (D_tip - D_root)*(x / (b/2))
        t = t_root + (t_tip - t_root)*(x / (b/2))
        
        d = D - 2 * t
        
        if np.any(d <= 0):
            raise ValueError('Inner dimension d must be positive.')
        
        # Moment of inertia calculation
        IBoom = (1/12) * (D**4 - d**4)
        AreaBoom = D**2 - d**2
        h = (H * cChord - D) / 2
        I = 2 * (IBoom + AreaBoom * h**2)
        
        outputs['D'] = D
        outputs['t'] = t
        outputs['d'] = d
        outputs['I'] = I
