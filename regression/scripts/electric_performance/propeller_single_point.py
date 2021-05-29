# propeller_single_point.py
#
# Created: Jan 2021, J. Smart
# Modified:

#-------------------------------------------------------------------------------
# Imports
#_______________________________________________________________________________

import SUAVE

from SUAVE.Core import Units, Data
from SUAVE.Methods.Performance.propeller_single_point import propeller_single_point

import numpy as np

import sys
sys.path.append('../Vehicles')

from X57_Maxwell import vehicle_setup

#-------------------------------------------------------------------------------
# Test Function
#-------------------------------------------------------------------------------

def main():

    vehicle = vehicle_setup()

    analyses = SUAVE.Analyses.Vehicle()
    atmosphere = SUAVE.Analyses.Atmospheric.US_Standard_1976()
    atmosphere.features.planet = SUAVE.Analyses.Planets.Planet()
    analyses.append(atmosphere)


    results = propeller_single_point(vehicle.propulsors.battery_propeller,
                                     analyses,
                                     pitch=0.,
                                     omega=1500. * Units.rpm,
                                     altitude= 5000. * Units.ft,
                                     delta_isa=0.,
                                     speed=10 * Units['m/s'],
                                     plots=True,
                                     print_results=True
                                     )

    thrust  = results.thrust
    torque  = results.torque
    power   = results.power
    Cp      = results.power_coefficient
    etap    = results.efficiency

    thrust_r    = 2396.2197771988695
    torque_r    = 855.5293272946128
    power_r     = 134386.23247796868
    Cp_r        = 0.3039521219476524
    etap_r      = 0.17830842736005018

    assert (np.abs(thrust - thrust_r) / thrust_r < 1e-6), "Propeller Single Point Regression Failed at Thrust Test"
    assert (np.abs(torque - torque_r) / torque_r < 1e-6), "Propeller Single Point Regression Failed at Torque Test"
    assert (np.abs(power - power_r) / power_r < 1e-6), "Propeller Single Point Regression Failed at Power Test"
    assert (np.abs(Cp - Cp_r) / Cp_r < 1e-6), "Propeller Single Point Regression Failed at Power Coefficient Test"
    assert (np.abs(etap - etap_r) / etap_r < 1e-6), "Propeller Single Point Regression Failed at Efficiency Test"

    return

if __name__ == '__main__':
    main()

    print('Propeller Single Point Regression Passed.')