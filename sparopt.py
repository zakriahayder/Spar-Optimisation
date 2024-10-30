import numpy as np
import openmdao.api as om

# 1. Define the Wing Geometry Component
class WingGeometry(om.ExplicitComponent):
    def initialize(self):
        self.options.declare('num_points', types=int, default=100, desc='Number of spanwise discretization points')
    
    def setup(self):
        n = self.options['num_points']
        
        
        # Inputs
        self.add_input('b', val=11.23, desc='Wingspan [m]')
        self.add_input('S', val=22.48, desc='Wing area [m^2]')
        
        # Outputs
        self.add_output('x', val=np.zeros(n), desc='Spanwise positions [m]')
        self.add_output('cChord', val=np.zeros(n), desc='Chord length at each spanwise position [m]')
        self.add_output('cRoot', val=0.0, desc='Root chord length [m]')
        
        # Declare partials
        self.declare_partials('*', '*', method='fd')
    
    def compute(self, inputs, outputs):
        n = self.options['num_points']
        b = inputs['b']
        S = inputs['S']
        
        # Spanwise positions
        x = np.linspace(0, b/2, n)
        outputs['x'] = x
        
        # Root chord length
        cRoot = (4 * S) / (np.pi * b)
        outputs['cRoot'] = cRoot
        
        # Chord length at each spanwise position
        outputs['cChord'] = cRoot * np.sqrt(1 - (2 * x / b)**2)

# 2. Define the Lift Distribution Component
class LiftDistribution(om.ExplicitComponent):
    def initialize(self):
        self.options.declare('num_points', types=int, default=100)
    
    def setup(self):
        n = self.options['num_points']
        
        # Inputs
        self.add_input('x', val=np.zeros(n), desc='Spanwise positions [m]')
        self.add_input('b', val=11.23, desc='Wingspan [m]')
        self.add_input('L', val=0.0, desc='Total lift force [N]')
        
        # Outputs
        self.add_output('Lprime', val=np.zeros(n), desc='Lift load distribution [N/m]')
        
        # Declare partials
        self.declare_partials('*', '*', method='fd')
    
    def compute(self, inputs, outputs):
        x = inputs['x']
        b = inputs['b']
        L = inputs['L']
        
        outputs['Lprime'] = (4 * L) / (b * np.pi) * np.sqrt(1 - (2 * x / b)**2)

# 3. Define the Spar Geometry Component
class SparGeometry(om.ExplicitComponent):
    def initialize(self):
        self.options.declare('num_points', types=int, default=100)
    
    def setup(self):
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
        
        # Declare partials
        self.declare_partials('*', '*', method='fd')
    
    def compute(self, inputs, outputs):
        x = inputs['x']
        n = len(x)
        b = inputs['b']
        D_root = inputs['D_root']
        D_tip = inputs['D_tip']
        t_root = inputs['t_root']
        t_tip = inputs['t_tip']
        H = inputs['H']
        cChord = inputs['cChord']
        
        # Linear variation of D and t
        D = D_root + (D_tip - D_root)*(x / (b/2))
        t = t_root + (t_tip - t_root)*(x / (b/2))
        
        # Inner dimension d
        d = D - 2 * t
        
        # Ensure d > 0
        if np.any(d <= 0):
            raise ValueError('Inner dimension d must be positive.')
        
        # Moment of inertia I(x)
        IBoom = (1/12) * (D**4 - d**4)
        AreaBoom = D**2 - d**2
        h = (H * cChord - D) / 2
        I = 2 * (IBoom + AreaBoom * h**2)
        
        # Outputs
        outputs['D'] = D
        outputs['t'] = t
        outputs['d'] = d
        outputs['I'] = I

# 4. Define the Structural Analysis Component
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

# 6. Define the Spar Optimization Group
class SparOptimization(om.Group):
    def setup(self):
        n = 100  # Number of discretization points

        # Add an IndepVarComp for shared inputs
        indeps = self.add_subsystem('indeps', om.IndepVarComp(), promotes=['*'])
        indeps.add_output('b', val=11.23)
        indeps.add_output('S', val=22.48)
        indeps.add_output('L', val=3000 * 9.81)
        indeps.add_output('E', val=100e9)
        indeps.add_output('H', val=0.13)
        indeps.add_output('rho_material', val=2700)

        # Add subsystems
        self.add_subsystem('wing_geom', WingGeometry(num_points=n))
        self.add_subsystem('lift_dist', LiftDistribution(num_points=n))
        self.add_subsystem('spar_geom', SparGeometry(num_points=n))
        self.add_subsystem('struct_analysis', StructuralAnalysis(num_points=n))
        self.add_subsystem('spar_mass', SparMass(num_points=n))

        # Connect components
        self.connect('b', ['wing_geom.b', 'lift_dist.b', 'spar_geom.b'])
        self.connect('S', 'wing_geom.S')
        self.connect('L', 'lift_dist.L')
        self.connect('E', 'struct_analysis.E')
        self.connect('H', ['spar_geom.H', 'struct_analysis.H'])
        self.connect('rho_material', 'spar_mass.rho_material')

        self.connect('wing_geom.x', ['lift_dist.x', 'spar_geom.x', 'struct_analysis.x', 'spar_mass.x'])
        self.connect('wing_geom.cChord', ['spar_geom.cChord', 'struct_analysis.cChord'])

        self.connect('lift_dist.Lprime', 'struct_analysis.Lprime')
        self.connect('spar_geom.I', 'struct_analysis.I')
        self.connect('spar_geom.D', 'spar_mass.D')
        self.connect('spar_geom.d', 'spar_mass.d')

        # Expose design variables
        self.add_design_var('spar_geom.D_root', lower=0.05, upper=0.5)
        self.add_design_var('spar_geom.D_tip', lower=0.05, upper=0.5)
        self.add_design_var('spar_geom.t_root', lower=0.005, upper=0.1)
        self.add_design_var('spar_geom.t_tip', lower=0.005, upper=0.1)

        # Add objective
        self.add_objective('spar_mass.mass')

        # Add constraints
        self.add_constraint('struct_analysis.sigma', upper=250e6)
        self.add_constraint('struct_analysis.w', upper=0.1)

# 7. Set up the Problem and Run the Optimization
def run_optimization():
    prob = om.Problem()
    prob.model = SparOptimization()
    
    # Set up the optimizer
    prob.driver = om.ScipyOptimizeDriver()
    prob.driver.options['optimizer'] = 'SLSQP'
    prob.driver.options['tol'] = 1e-6
    prob.driver.options['maxiter'] = 1000
    
    # Set up the problem
    prob.setup()
    
    # Set initial values for design variables
    prob.set_val('spar_geom.D_root', 0.25)
    prob.set_val('spar_geom.D_tip', 0.15)
    prob.set_val('spar_geom.t_root', 0.02)
    prob.set_val('spar_geom.t_tip', 0.01)
    
    # Run the optimization
    prob.run_driver()
    
    # Get the optimized results - extract scalar values from arrays
    D_root_opt = float(prob.get_val('spar_geom.D_root')[0])
    D_tip_opt = float(prob.get_val('spar_geom.D_tip')[0])
    t_root_opt = float(prob.get_val('spar_geom.t_root')[0])
    t_tip_opt = float(prob.get_val('spar_geom.t_tip')[0])
    mass_opt = float(prob.get_val('spar_mass.mass')[0])
    
    print(f"Optimized D_root: {D_root_opt:.4f} m")
    print(f"Optimized D_tip: {D_tip_opt:.4f} m")
    print(f"Optimized t_root: {t_root_opt:.4f} m")
    print(f"Optimized t_tip: {t_tip_opt:.4f} m")
    print(f"Minimum mass of spar: {mass_opt:.2f} kg")
    
    # Optional: Plotting results
    import matplotlib.pyplot as plt
    x = prob.get_val('wing_geom.x')
    sigma = prob.get_val('struct_analysis.sigma')
    w = prob.get_val('struct_analysis.w')
    D = prob.get_val('spar_geom.D')
    t = prob.get_val('spar_geom.t')
    
    plt.figure(figsize=(12, 8))
    
    plt.subplot(2, 2, 1)
    plt.plot(x, sigma / 1e6)
    plt.xlabel('Spanwise Position [m]')
    plt.ylabel('Stress [MPa]')
    plt.title('Stress Distribution')
    plt.grid(True)
    
    plt.subplot(2, 2, 2)
    plt.plot(x, w * 1000)
    plt.xlabel('Spanwise Position [m]')
    plt.ylabel('Deflection [mm]')
    plt.title('Deflection')
    plt.grid(True)
    
    plt.subplot(2, 2, 3)
    plt.plot(x, D * 1000)
    plt.xlabel('Spanwise Position [m]')
    plt.ylabel('Outer Dimension D [mm]')
    plt.title('Optimized Outer Dimension')
    plt.grid(True)
    
    plt.subplot(2, 2, 4)
    plt.plot(x, t * 1000)
    plt.xlabel('Spanwise Position [m]')
    plt.ylabel('Wall Thickness t [mm]')
    plt.title('Optimized Wall Thickness')
    plt.grid(True)
    
    plt.tight_layout()
    plt.show()

# Run the optimization
if __name__ == '__main__':
    run_optimization()
