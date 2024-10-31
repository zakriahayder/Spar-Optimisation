import openmdao.api as om
import numpy as np

# 5. Define the Spar Mass Component
class SparMass(om.ExplicitComponent):
    def initialize(self):
        self.options.declare('num_points', types=int, default=100)
    
    def setup(self):
        n = self.options['num_points']
        
        # Inputs
        self.add_input('x', val=np.zeros(n), desc='Spanwise positions [m]')
        self.add_input('D', val=np.zeros(n), desc='Outer dimension along the span [m]')
        self.add_input('d', val=np.zeros(n), desc='Inner dimension along the span [m]')
        self.add_input('rho_material', val=2700.0, desc='Material density [kg/m^3]')
        
        # Outputs
        self.add_output('mass', val=0.0, desc='Total mass of the spar [kg]')
        
        # Declare partials
        self.declare_partials('*', '*', method='fd')
    
    def compute(self, inputs, outputs):
        x = inputs['x']
        D = inputs['D']
        d = inputs['d']
        rho_material = inputs['rho_material']
        
        # Cross-sectional area A(x)
        A = D**2 - d**2
        
        # Volume V and mass
        V = 2 * np.trapz(A, x)  # Multiply by 2 for both wings
        mass = V * rho_material
        
        outputs['mass'] = mass