import openmdao.api as om
import numpy as np

class StructuralAnalysis(om.ExplicitComponent):
    def initialize(self):
        self.options.declare('num_points', types=int, default=100)
    
    def setup(self):
        n = self.options['num_points']
        
        # Inputs
        self.add_input('x', val=np.zeros(n), desc='Spanwise positions [m]')
        self.add_input('Lprime', val=np.zeros(n), desc='Lift load distribution [N/m]')
        self.add_input('I', val=np.zeros(n), desc='Moment of inertia along the span [m^4]')
        self.add_input('E', val=100e9, desc='Young\'s modulus [Pa]')
        self.add_input('H', val=0.13, desc='Fraction of chord for spar location')
        self.add_input('cChord', val=np.zeros(n), desc='Chord length at each spanwise position [m]')
        
        # Outputs
        self.add_output('V', val=np.zeros(n), desc='Shear force distribution [N]')
        self.add_output('M', val=np.zeros(n), desc='Bending moment distribution [Nm]')
        self.add_output('theta', val=np.zeros(n), desc='Deflection angle distribution [rad]')
        self.add_output('w', val=np.zeros(n), desc='Vertical deflection distribution [m]')
        self.add_output('sigma', val=np.zeros(n), desc='Stress distribution [Pa]')
        
        # Declare partials
        self.declare_partials('*', '*', method='fd')
    
    def compute(self, inputs, outputs):
        x = inputs['x']
        Lprime = inputs['Lprime']
        I = inputs['I']
        E = inputs['E']
        H = inputs['H']
        cChord = inputs['cChord']
        n = len(x)
        
        # Reaction forces
        A_y = np.trapz(Lprime, x)
        M_A = np.trapz(Lprime * x, x)
        
        # Shear force V(x)
        V = -A_y + np.cumsum(Lprime * np.gradient(x))
        
        # Bending moment M(x)
        M = M_A + np.cumsum(V * np.gradient(x))
        
        # Deflection angle theta(x)
        theta = np.cumsum(M / (E * I) * np.gradient(x))
        
        # Vertical deflection w(x)
        w = np.cumsum(theta * np.gradient(x))
        
        # Stress distribution sigma(x)
        sigma = 0.5 * H * cChord * M / I
        
        # Outputs
        outputs['V'] = V
        outputs['M'] = M
        outputs['theta'] = theta
        outputs['w'] = w
        outputs['sigma'] = sigma