[![GitHub top language](https://img.shields.io/github/languages/top/mherrmann21/LGVI-Multibody-Experiments)](https://matlab.mathworks.com/)
![GitHub Repo stars](https://img.shields.io/github/stars/mherrmann21/LGVI-Multibody-Experiments?style=social)
[![License](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)
![MATLAB Version](https://img.shields.io/badge/MATLAB-R2025b%2B-blue)
![Work in Progress](https://img.shields.io/badge/status-WIP-orange)

# LGVI Multibody Experiments
Numerical experiments for the analysis of time integrators &amp; discretizations for the simulation &amp; optimal control of rigid-flexible robotic systems.

This repository contains the numerical experiments on time integration and optimal control of rigid-flexible multibody systems reported in [1].
In the experiments, various common ODE integrators for nonstiff and stiff systems are compared to a Lie-group variational integrator in terms of numerical properties and computational efficiency.
The implementation is based on the MATLAB toolbox [ELARA](https://github.com/ELARA-Toolbox/ELARA), which must be installed on the machine.

#### Multibody Simulation Experiments

1. Free motion of a planar four-link pendulum
2. Nonlinear oscillations of a geometrically exact cantilever beam (modeled as a Kirchhoff beam); this is the same case also treated in [2, 3]
3. Robot manipulator with a highly flexible end link under closed-loop PD control

The first two simulation cases are each analyzed both with and without dissipation.

#### Optimal Control Experiments

1. Rest-to-rest motion of a planar robot manipulator (after [4])
2. TCP trajectory tracking of a three-DOF robot manipulator
3. TCP trajectory tracking of a tendon-driven continuum manipulator

## Running the Simulation Studies

1. Make sure the `mex` files in the toolbox have been built correctly (run `elara.build` and check with `elara.setup`)

2. Run `startup_sim_studies.m` to add all required functions to the MATLAB path and check for dependencies.

3. Run the simulation studies in the `/studies` folder:
   - `sim_study_integrators_run_sims`: Time integration simulation study
   - `sim_study_OCP_disc_run_sims`: Optimal control discretization study
   
   The scripts save the simulation results in the `/results/` folders.

4. Run the evaluation scripts to generate the output plots.
Adjust the system, dissipation case, and data file paths in the script settings.

### Additional Scripts

Under `/tests`, there are two additional validation scripts that perform simulations of the four-link pendulum and the cantilever beam of the integrator study.
Additionally, `validation_sim_cantilever_beam.m` compares the simulation results to the literature results from [3].


## Requirements

* MATLAB R2025b
* [ELARA](https://github.com/ELARA-Toolbox/ELARA) Toolbox, V0.1 (installed and available on the MATLAB path)
* [CasADi](https://web.casadi.org/) V3.7.2  (only for the optimal control experiments; must be installed and available on the MATLAB path)
* [Coin-HSL linear solvers](https://licences.stfc.ac.uk/product/coin-hsl) for IPOPT (version 2024.05.15)

Later versions of the required software may work, but have not been tested and may lead to different results.

The code is tested on Windows 11.

### Additional Software Used

* This repository includes the RADAU integrator implemented by Denis Bichsel. It is originally available [here](https://www.unige.ch/~hairer/software.html).

* The [MATLAB version](https://github.com/chadagreene/crameri) of the scientific color maps [Crameri](https://www.fabiocrameri.ch/colourmaps/) is included as a submodule.

* The [PHCosseratRods](https://github.com/plkinon/ph_cosserat_rods/) repository (corresponding to [3]) is included as submodule for validation and comparison of the cantilever beam simulation case.

### Installing the HSL linear solvers for IPOPT

Download: https://licences.stfc.ac.uk/product/coin-hsl

Installation instructions: https://github.com/casadi/casadi/wiki/Obtaining-HSL

* Create an account on the HSL licensing website (academic license) and download binaries for Windows
* Add the \bin subfolder to the `PATH` variable

Now, the solvers should be available in CasADi/IPOPT:

      opts = struct;
      opts.ipopt.linear_solver = 'ma97';
      NLPSolver = nlpsol('solver', 'ipopt', NLprob, opts);


## Additional Information
For additional details on the implementation, see the ELARA documentation, and for background on the systems and integrators, see [1].

## License

The code in this repository is licensed under the MIT License; see [LICENSE](LICENSE).

Third-party components in `third-party/` are redistributed under their respective upstream terms, notices, and citation requirements.

## References

[1] M. Herrmann: Geometric Modeling and Optimal Control of Rigid-Flexible Robot Manipulators. PhD Thesis, Technical University of Munich, 2026 (in preparation).

[2] M. Herrmann and P. Kotyczka. “Relative-kinematic formulation of geometrically exact beam dynamics based on Lie group variational integrators”. In: Computer Methods in Applied Mechanics and Engineering 432 (2024), p. 117367. doi: 10.1016/j.cma.2024.117367

[3] P. L. Kinon, S. R. Eugster, and P. Betsch. “Mixed formulation and structure-preserving discretization of Cosserat rod dynamics in a port-Hamiltonian framework”. In: Computer Methods in Applied Mechanics and Engineering 458 (2026), p. 118966. doi: 10.1016/j.cma.2026.118966

[4] S. Ober-Blöbaum, O. Junge, and J. E. Marsden. “Discrete mechanics and optimal control: An analysis”. In: ESAIM: Control, Optimisation and Calculus of Variations 17.2 (2011), pp. 322–352. doi: 10.1051/cocv/2010012

