import matplotlib.pyplot as plt
import numpy as np
import openmdao.api as om
from lift_distribution import LiftDistribution
from spar_geometry import SparGeometry
from spar_mass import SparMass
from structural_analysis import StructuralAnalysis
from wing_geometry import WingGeometry


class SparOptimization(om.Group):
    """Spar Optimization Group using OpenMDAO"""
    def setup(self):
        n = 100  # Number of discretization points

        indeps = self.add_subsystem('indeps', om.IndepVarComp(), promotes=['*'])
        indeps.add_output('b', val=11.23)
        indeps.add_output('S', val=22.48)
        indeps.add_output('L', val=3000 * 9.81)
        indeps.add_output('E', val=100e9)
        indeps.add_output('H', val=0.13)
        indeps.add_output('rho_material', val=2700)

        self.add_subsystem('wing_geom', WingGeometry(num_points=n))
        self.add_subsystem('lift_dist', LiftDistribution(num_points=n))
        self.add_subsystem('spar_geom', SparGeometry(num_points=n))
        self.add_subsystem('struct_analysis', StructuralAnalysis(num_points=n))
        self.add_subsystem('spar_mass', SparMass(num_points=n))

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

        self.add_design_var('spar_geom.D_root', lower=0.05, upper=0.5)
        self.add_design_var('spar_geom.D_tip', lower=0.05, upper=0.5)
        self.add_design_var('spar_geom.t_root', lower=0.005, upper=0.1)
        self.add_design_var('spar_geom.t_tip', lower=0.005, upper=0.1)

        self.add_objective('spar_mass.mass')
        self.add_constraint('struct_analysis.sigma', upper=250e6)
        self.add_constraint('struct_analysis.w', upper=0.1)


def run_optimization():
    """Run the spar optimization"""
    prob = om.Problem()
    prob.model = SparOptimization()

    prob.driver = om.ScipyOptimizeDriver()
    prob.driver.options['optimizer'] = 'SLSQP'
    prob.driver.options['tol'] = 1e-6
    prob.driver.options['maxiter'] = 1000

    prob.setup()

    prob.set_val('spar_geom.D_root', 0.25)
    prob.set_val('spar_geom.D_tip', 0.15)
    prob.set_val('spar_geom.t_root', 0.02)
    prob.set_val('spar_geom.t_tip', 0.01)

    prob.run_driver()

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


if __name__ == '__main__':
    run_optimization()
