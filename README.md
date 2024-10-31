# Introduction

The objective of this project is to optimize the design of an spitfire aircraft
wing spar to minimize its mass while satisfying structural constraints
on stress and deflection. This involves applying **Multidisciplinary
Design Optimization (MDO)** techniques, utilizing Python programming,
and leveraging the **OpenMDAO** library for efficient computation.

# Wing Geometry Modeling

The wing has an elliptical planform, and the chord length *c*(*y*)
varies along the spanwise position *y* according to:

$$c(y) = c\_{\text{root}} \cdot \sqrt{1 - \left( \dfrac{2y}{b} \right)^2 }$$

where:

-   *c*<sub>root</sub> is the chord length at the root, calculated as:
    $$c\_{\text{root}} = \dfrac{4S}{\pi b}$$

-   *S* is the wing area.

-   *b* is the wingspan.

# Lift Load Distribution

The lift load per unit span *L*′(*y*) is distributed elliptically along
the wing:

$$L'(y) = \dfrac{4L}{b \pi} \cdot \sqrt{1 - \left( \dfrac{2y}{b} \right)^2 }$$

where *L* = *mg* is the total lift force (equal to the aircraft’s
weight).

# Spar Geometry

The spar is modeled using hollow square booms. The moment of inertia
*I*(*y*) of the spar cross-section at position *y* is given by:

*I*(*y*) = 2(*I*<sub>boom</sub> + *A*<sub>boom</sub>*h*<sup>2</sup>(*y*))

where:

-   $I\_{\text{boom}} = \dfrac{1}{12} \left( D^4(y) - d^4(y) \right)$ is
    the moment of inertia of a single boom.

-   *A*<sub>boom</sub> = *D*<sup>2</sup>(*y*) − *d*<sup>2</sup>(*y*) is
    the cross-sectional area of a boom.

-   *D*(*y*) and *d*(*y*) are the outer and inner dimensions of the
    square boom at position *y*.

-   $h(y) = \dfrac{H(y) - D(y)}{2}$ is the distance from the neutral
    axis to the center of the boom.

-   *H*(*y*) = *H* ⋅ *c*(*y*) is the spar height, with *H* being the
    fraction of the chord where the spar is located.

# Structural Analysis

Using Euler-Bernoulli beam theory, the following quantities are
calculated along the span:

## Shear Force *V*(*y*)

*V*(*y*) = −*A*<sub>*y*</sub> + ∫<sub>0</sub><sup>*y*</sup>*L*′(*s*) *ds*

where *A*<sub>*y*</sub> is the reaction force at the root.

## Bending Moment *M*(*y*)

*M*(*y*) = *M*<sub>*A*</sub> + ∫<sub>0</sub><sup>*y*</sup>*V*(*s*) *ds*

where *M*<sub>*A*</sub> is the reaction moment at the root.

## Deflection Angle *θ*(*y*)

$$\theta(y) = \int_0^y \dfrac{M(s)}{E I(s)} \\ ds$$

## Vertical Deflection *w*(*y*)

*w*(*y*) = ∫<sub>0</sub><sup>*y*</sup>*θ*(*s*) *ds*

## Stress *σ*(*y*)

$$\sigma(y) = \dfrac{M(y) H(y) c(y)}{2 I(y)}$$

where:

-   *E* is Young’s modulus of the spar material.

-   *c*(*y*) is the chord length at position *y*.

# Optimization Problem

The optimization aims to minimize the total mass of the spar while
satisfying stress and deflection constraints.

## Objective Function

Minimize *m* = 2∫<sub>0</sub><sup>*b*/2</sup>*ρ*<sub>material</sub> ⋅ *A*<sub>boom</sub>(*y*) *dy*

where:

-   *ρ*<sub>material</sub> is the material density.

-   The factor of 2 accounts for both wings.

## Design Variables

-   Outer dimension at root *D*<sub>root</sub>

-   Outer dimension at tip *D*<sub>tip</sub>

-   Wall thickness at root *t*<sub>root</sub>

-   Wall thickness at tip *t*<sub>tip</sub>

The outer dimension and wall thickness vary linearly along the span:

$$D(y) = D\_{\text{root}} + \left( D\_{\text{tip}} - D\_{\text{root}} \right) \left( \dfrac{2y}{b} \right)$$

$$t(y) = t\_{\text{root}} + \left( t\_{\text{tip}} - t\_{\text{root}} \right) \left( \dfrac{2y}{b} \right)$$

## Constraints

#### Stress Constraint

*σ*(*y*) ≤ *σ*<sub>allowable</sub>

#### Deflection Constraint

$$w\left( \dfrac{b}{2} \right) \leq w\_{\text{allowable}}$$

# Input Parameters

The following input parameters were used in the optimization:

-   Wingspan, *b* = 11.23 m

-   Wing area, *S* = 22.48 m<sup>2</sup>

-   Aircraft mass, *m* = 3000 kg

-   Total lift force, *L* = *m**g* = 3000 × 9.81 N

-   Young’s modulus, *E* = 100 GPa

-   Spar location fraction, *H* = 0.13

-   Material density, *ρ*<sub>material</sub> = 2700 kg/m<sup>3</sup>

-   Allowable stress, *σ*<sub>allowable</sub> = 250 MPa

-   Allowable deflection, *w*<sub>allowable</sub> = 0.1 m

# Implementation with OpenMDAO

The problem is implemented in Python using the **OpenMDAO** framework,
which facilitates MDO by providing tools for defining components,
connecting them, and performing optimization.

## Components

1.  **WingGeometry**: Calculates chord lengths and spanwise positions.

2.  **LiftDistribution**: Computes lift load distribution along the
    wing.

3.  **SparGeometry**: Defines spar dimensions along the span.

4.  **StructuralAnalysis**: Performs calculations of shear force,
    bending moment, deflection, and stress.

5.  **SparMass**: Computes the total mass of the spar.

## Optimization Setup

-   **Design Variables**:

    -   *D*<sub>root</sub>, *D*<sub>tip</sub>, *t*<sub>root</sub>,
        *t*<sub>tip</sub>

-   **Objective**:

    -   Minimize spar mass.

-   **Constraints**:

    -   Stress at all spanwise positions must not exceed
        *σ*<sub>allowable</sub> = 250 MPa.

    -   Tip deflection must not exceed *w*<sub>allowable</sub> = 0.1 m.

## OpenMDAO Code Structure

#### Defining Components

Each component is defined as a class inheriting from
`om.ExplicitComponent`, with inputs and outputs specified.

    class WingGeometry(om.ExplicitComponent):
        def setup(self):
            self.add_input('b', val=11.23)
            self.add_input('S', val=22.48)
            self.add_output('x', val=np.zeros(n))
            self.add_output('cChord', val=np.zeros(n))

#### Connecting Components

Components are connected within the `SparOptimization` group.

    self.connect('wing_geom.x', ['lift_dist.x', 'spar_geom.x', 'struct_analysis.x'])
    self.connect('wing_geom.cChord', ['spar_geom.cChord', 'struct_analysis.cChord'])

#### Defining the Objective and Constraints

    self.add_design_var('spar_geom.D_root', lower=0.05, upper=0.5)
    self.add_design_var('spar_geom.D_tip', lower=0.05, upper=0.5)
    self.add_design_var('spar_geom.t_root', lower=0.005, upper=0.1)
    self.add_design_var('spar_geom.t_tip', lower=0.005, upper=0.1)
    self.add_objective('spar_mass.mass')
    self.add_constraint('struct_analysis.sigma', upper=250e6)
    self.add_constraint('struct_analysis.w', upper=0.1)

# Results

After running the optimization, the following results were obtained:

## Optimized Dimensions

-   Optimized outer dimension at root: *D*<sub>root</sub> = 0.0500 m

-   Optimized outer dimension at tip: *D*<sub>tip</sub> = 0.0500 m

-   Optimized wall thickness at root: *t*<sub>root</sub> = 0.0050 m

-   Optimized wall thickness at tip: *t*<sub>tip</sub> = 0.0050 m

## Minimum Spar Mass

-   Minimum mass of spar: *m* = 27.29 kg

## Optimization Output

The optimization process yielded the following output:

    Optimization terminated successfully    (Exit mode 0)
                Current function value: 27.288900000000012
                Iterations: 3
                Function evaluations: 2
                Gradient evaluations: 2
    Optimization Complete

## Stress and Deflection Profiles

The optimized design satisfies all constraints:

-   The maximum stress along the span is below the allowable stress of
    250 MPa.

-   The tip deflection is less than the allowable deflection of 0.1 m.

# Comparison with MATLAB Results

For validation, the optimization was also performed using MATLAB,
yielding similar results:

-   Optimized outer dimension at root: *D*<sub>root</sub> = 0.0500 m

-   Optimized outer dimension at tip: *D*<sub>tip</sub> = 0.0500 m

-   Optimized wall thickness at root: <sub>root</sub> = 0.0050 m

-   Optimized wall thickness at tip: *t*<sub>tip</sub> = 0.0050 m

-   Minimum mass of spar: *m* = 27.29 kg

## MATLAB Optimization Output

     Iter  Func-count            Fval   Feasibility   Step Length       Norm of   First-order  
                                                                           step    optimality
        0           5    3.456613e+02     0.000e+00     1.000e+00     0.000e+00     1.112e+04  
        1          10    2.728890e+01     1.388e-17     1.000e+00     2.242e-01     8.692e+03  
        2          11    2.728890e+01     1.388e-17     7.000e-01     1.377e-17     1.388e-17

# Conclusion

This project demonstrates the application of **Multidisciplinary Design
Optimization** techniques using Python and OpenMDAO to optimize the
design of a wing spar. The optimization successfully minimized the spar
mass while satisfying all structural constraints, and the results were
consistent with those obtained from MATLAB, validating the approach.

# Variables and Constants Used

-   *b*: Wingspan (m)

-   *S*: Wing area (m<sup>2</sup>)

-   *c*(*y*): Chord length at spanwise position *y* (m)

-   *c*<sub>root</sub>: Root chord length (m)

-   *L*: Total lift force (N)

-   *L*′(*y*): Lift load per unit span at position *y* (N/m)

-   *D*(*y*): Outer dimension of the spar at position *y* (m)

-   *d*(*y*): Inner dimension of the spar at position *y* (m)

-   *t*(*y*): Wall thickness of the spar at position *y* (m)

-   *H*(*y*): Spar height at position *y* (m)

-   *I*(*y*): Moment of inertia at position *y* (m<sup>4</sup>)

-   *M*(*y*): Bending moment at position *y* (Nm)

-   *V*(*y*): Shear force at position *y* (N)

-   *θ*(*y*): Deflection angle at position *y* (rad)

-   *w*(*y*): Vertical deflection at position *y* (m)

-   *σ*(*y*): Stress at position *y* (Pa)

-   *E*: Young’s modulus (Pa)

-   *ρ*<sub>material</sub>: Material density (kg/m<sup>3</sup>)

-   *σ*<sub>allowable</sub>: Allowable stress (Pa)

-   *w*<sub>allowable</sub>: Allowable deflection (m)

# Assumptions

-   The wing is modeled as an idealized structure with an elliptical
    lift distribution.

-   Material properties are homogeneous and isotropic.

-   The spar dimensions vary linearly from root to tip.

-   The analysis uses Euler-Bernoulli beam theory, assuming small
    deflections.
