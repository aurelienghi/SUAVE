# Vehicles.py
# 
# Created:  Feb. 2016, M. Vegh
# Modified: Aug. 2017, E. Botero
# Modified: Jan. 2024, A. Ghiglino

# ----------------------------------------------------------------------        
#   Imports
# ----------------------------------------------------------------------    

import numpy as np

import SUAVE
from SUAVE.Core import Units
from SUAVE.Methods.Propulsion.turbofan_sizing import turbofan_sizing
from SUAVE.Methods.Geometry.Two_Dimensional.Planform import wing_planform

from copy import deepcopy

# ----------------------------------------------------------------------
#   Define the Vehicle
# ----------------------------------------------------------------------

def setup():
    
    base_vehicle = base_setup()
    configs = configs_setup(base_vehicle)
    
    return configs

def base_setup():

    # ------------------------------------------------------------------
    #   Initialize the Vehicle
    # ------------------------------------------------------------------

    vehicle = SUAVE.Vehicle()
    vehicle.tag = 'Airbus_A380'

    # ------------------------------------------------------------------
    #   Vehicle-level Properties
    # ------------------------------------------------------------------

    vehicle.mass_properties.max_takeoff               = 575000.   # kg
    vehicle.mass_properties.operating_empty           = 285000.   # kg
    vehicle.mass_properties.takeoff                   = 548500.   # kg 575 pax
    vehicle.mass_properties.max_zero_fuel             = 369000.   # kg
    vehicle.mass_properties.max_payload               = 84000.   # kg
    vehicle.mass_properties.max_fuel                  = 253983.   # kg
    vehicle.mass_properties.cargo                     =     0.0  # kg

    # envelope properties
    vehicle.envelope.ultimate_load = 3.5
    vehicle.envelope.limit_load    = 1.5

    # basic parameters
    vehicle.reference_area         = 845.
    vehicle.passengers             = 575
    vehicle.systems.control        = "fully powered"
    vehicle.systems.accessories    = "long range"


    # ------------------------------------------------------------------
    #   Main Wing
    # ------------------------------------------------------------------
    wing                         = SUAVE.Components.Wings.Main_Wing()
    wing.tag                     = 'main_wing'
    wing.areas.reference         = 845.0
    wing.aspect_ratio            = 7.54
    wing.chords.root             = 17.7
    wing.chords.tip              = 4.0
    wing.sweeps.quarter_chord    = 23.0 * Units.deg
    wing.thickness_to_chord      = 0.11
    wing.taper                   = 0.225
    wing.dihedral                = 5.00 * Units.deg
    wing.spans.projected         = 28.72
    wing.origin                  = [[34,0,-2.50]]
    wing.vertical                = False
    wing.symmetric               = True       
    wing.high_lift               = True
    wing.areas.exposed           = 0.80 * wing.areas.wetted        
    wing.twists.root             = 2.0 * Units.degrees
    wing.twists.tip              = 0.0 * Units.degrees    
    wing.dynamic_pressure_ratio  = 1.0
    
    # control surfaces -------------------------------------------
    flap                       = SUAVE.Components.Wings.Control_Surfaces.Flap() 
    flap.tag                   = 'flap' 
    flap.span_fraction_start   = 0.11
    flap.span_fraction_end     = 0.85
    flap.deflection            = 0.0 * Units.deg 
    flap.chord_fraction        = 0.28    
    flap.configuration_type    = 'double_slotted'
    wing.append_control_surface(flap)   
        
    slat                       = SUAVE.Components.Wings.Control_Surfaces.Slat()
    slat.tag                   = 'slat' 
    slat.span_fraction_start   = 0.324 
    slat.span_fraction_end     = 0.963     
    slat.deflection            = 1.0 * Units.deg 
    slat.chord_fraction        = 0.1   
    wing.append_control_surface(slat) 
    
    wing                         = wing_planform(wing)
    
    wing.areas.exposed           = 0.80 * wing.areas.wetted
    wing.twists.root             = 2.0 * Units.degrees
    wing.twists.tip              = 0.0 * Units.degrees    
    wing.dynamic_pressure_ratio  = 1.0   

    # add to vehicle
    vehicle.append_component(wing)
    
    # ------------------------------------------------------------------
    #  Horizontal Stabilizer
    # ------------------------------------------------------------------

    wing = SUAVE.Components.Wings.Horizontal_Tail()
    wing.tag = 'horizontal_stabilizer'
    wing.areas.reference         = 182.0
    wing.aspect_ratio            = 5.0
    wing.sweeps.quarter_chord    = 34.5 * Units.deg
    wing.thickness_to_chord      = 0.11
    wing.taper                   = 0.21
    wing.dihedral                = 8.4 * Units.degrees
    wing.origin                  = [[65,0,0.80]]
    wing.vertical                = False
    wing.symmetric               = True       
    wing.high_lift               = False  
    wing                         = wing_planform(wing)
    wing.areas.exposed           = 0.9 * wing.areas.wetted 
    wing.twists.root             = 2.0 * Units.degrees
    wing.twists.tip              = 2.0 * Units.degrees    
    wing.dynamic_pressure_ratio  = 0.90

    # add to vehicle
    vehicle.append_component(wing)

    # ------------------------------------------------------------------
    #   Vertical Stabilizer
    # ------------------------------------------------------------------

    wing = SUAVE.Components.Wings.Vertical_Tail()
    wing.tag = 'vertical_stabilizer'
    wing.areas.reference         = 116.0
    wing.aspect_ratio            =  1.7
    wing.sweeps.quarter_chord    = 35. * Units.deg
    wing.thickness_to_chord      = 0.11
    wing.taper                   = 0.40
    wing.dihedral                = 0.00
    wing.origin                  = [[60,0,4.0]]
    wing.vertical                = True
    wing.symmetric               = False       
    wing.high_lift               = False
    wing                         = wing_planform(wing)
    wing.areas.exposed           = 0.9 * wing.areas.wetted
    wing.twists.root             = 0.0 * Units.degrees
    wing.twists.tip              = 0.0 * Units.degrees    
    wing.dynamic_pressure_ratio  = 1.00
    
    # add to vehicle
    vehicle.append_component(wing)
    
    # ------------------------------------------------------------------
    #  Fuselage
    # ------------------------------------------------------------------

    fuselage = SUAVE.Components.Fuselages.Fuselage()
    fuselage.tag    = 'fuselage'
    fuselage.origin = [[37,0,0]]
    fuselage.number_coach_seats    = vehicle.passengers
    fuselage.seats_abreast         = 10
    fuselage.seat_pitch            = 30. * Units.inches

    fuselage.fineness.nose         = 1.28
    fuselage.fineness.tail         = 3.48
    fuselage.lengths.nose          = 8.0
    fuselage.lengths.tail          = 20.0
    fuselage.lengths.cabin         = 44.73
    fuselage.lengths.total         = 72.73
    fuselage.lengths.fore_space    = 0.
    fuselage.lengths.aft_space     = 0.

    fuselage.width                 = 8.0 * Units.meters

    fuselage.heights.maximum       = 8.3    
    fuselage.heights.at_quarter_length          = 8.3 
    fuselage.heights.at_three_quarters_length   = 8.3 
    fuselage.heights.at_wing_root_quarter_chord = 8.3 

    fuselage.areas.side_projected  = 560
    fuselage.areas.wetted          = 560
    fuselage.areas.front_projected = 50.0

    fuselage.effective_diameter    = 8.0

    fuselage.differential_pressure = 10**5 * Units.pascal    # Maximum differential pressure

    # add to vehicle
    vehicle.append_component(fuselage)
    
    

    # -----------------------------------------------------------------
    # Design the Nacelle
    # ----------------------------------------------------------------- 
    nacelle                       = SUAVE.Components.Nacelles.Nacelle()
    nacelle.diameter              = 3.4
    nacelle.length                = 4.0
    nacelle.tag                   = 'nacelle_1'
    nacelle.inlet_diameter        = 3.0
    nacelle.origin                = [[22.0,15.,-2.0]]
    Awet                          = 1.1*np.pi*nacelle.diameter*nacelle.length # 1.1 is simple coefficient
    nacelle.areas.wetted          = Awet 
    nacelle_airfoil               = SUAVE.Components.Airfoils.Airfoil() 
    nacelle_airfoil.naca_4_series_airfoil = '2410'
    nacelle.append_airfoil(nacelle_airfoil) 

    nacelle_2                     = deepcopy(nacelle)
    nacelle_2.tag                 = 'nacelle_2'
    nacelle_2.origin              = [[22.0,-15.,-2.0]]
    
    nacelle_3                     = deepcopy(nacelle)
    nacelle_3.tag                 = 'nacelle_3'
    nacelle_3.origin              = [[30.0,25.,0.0]]
    
    nacelle_4                     = deepcopy(nacelle)
    nacelle_4.tag                 = 'nacelle_4'
    nacelle_4.origin              = [[30.0,-25.,0.0]]
    
    
    vehicle.append_component(nacelle)   
    vehicle.append_component(nacelle_2)
    vehicle.append_component(nacelle_3)    
    vehicle.append_component(nacelle_4)
    

    # ------------------------------------------------------------------
    #  Turbofan Network
    # ------------------------------------------------------------------    

    #initialize the gas turbine network
    gt_engine                   = SUAVE.Components.Energy.Networks.Turbofan()
    gt_engine.tag               = 'turbofan'
    gt_engine.origin            = [[22.0,15.,-2.0],[22.0,-15.,-2.0],[30.0,25.,0.0],[30.0,-25.,0.0]]
    gt_engine.number_of_engines = 4
    gt_engine.bypass_ratio      = 9.6

    #add working fluid to the network
    gt_engine.working_fluid     = SUAVE.Attributes.Gases.Air()


    #Component 1 : ram,  to convert freestream static to stagnation quantities
    ram           = SUAVE.Components.Energy.Converters.Ram()
    ram.tag       = 'ram'
    #add ram to the network
    gt_engine.ram = ram


    #Component 2 : inlet nozzle
    inlet_nozzle                       = SUAVE.Components.Energy.Converters.Compression_Nozzle()
    inlet_nozzle.tag                   = 'inlet nozzle'
    inlet_nozzle.polytropic_efficiency = 0.98
    inlet_nozzle.pressure_ratio        = 0.98
    #add inlet nozzle to the network
    gt_engine.inlet_nozzle             = inlet_nozzle


    #Component 3 :low pressure compressor    
    low_pressure_compressor                       = SUAVE.Components.Energy.Converters.Compressor()    
    low_pressure_compressor.tag                   = 'lpc'
    low_pressure_compressor.polytropic_efficiency = 0.91
    low_pressure_compressor.pressure_ratio        = 1.9    
    #add low pressure compressor to the network    
    gt_engine.low_pressure_compressor             = low_pressure_compressor

    #Component 4 :high pressure compressor  
    high_pressure_compressor                       = SUAVE.Components.Energy.Converters.Compressor()    
    high_pressure_compressor.tag                   = 'hpc'
    high_pressure_compressor.polytropic_efficiency = 0.91
    high_pressure_compressor.pressure_ratio        = 10.0   
    #add the high pressure compressor to the network    
    gt_engine.high_pressure_compressor             = high_pressure_compressor

    #Component 5 :low pressure turbine  
    low_pressure_turbine                        = SUAVE.Components.Energy.Converters.Turbine()   
    low_pressure_turbine.tag                    ='lpt'
    low_pressure_turbine.mechanical_efficiency  = 0.99
    low_pressure_turbine.polytropic_efficiency  = 0.93
    #add low pressure turbine to the network     
    gt_engine.low_pressure_turbine              = low_pressure_turbine

    #Component 5 :high pressure turbine  
    high_pressure_turbine                       = SUAVE.Components.Energy.Converters.Turbine()   
    high_pressure_turbine.tag                   ='hpt'
    high_pressure_turbine.mechanical_efficiency = 0.99
    high_pressure_turbine.polytropic_efficiency = 0.93
    #add the high pressure turbine to the network    
    gt_engine.high_pressure_turbine             = high_pressure_turbine 

    #Component 6 :combustor  
    combustor                           = SUAVE.Components.Energy.Converters.Combustor()   
    combustor.tag                       = 'Comb'
    combustor.efficiency                = 0.99 
    combustor.alphac                    = 1.0     
    combustor.turbine_inlet_temperature = 1500
    combustor.pressure_ratio            = 0.95
    combustor.fuel_data                 = SUAVE.Attributes.Propellants.Jet_A()    
    #add the combustor to the network    
    gt_engine.combustor                 = combustor

    #Component 7 :core nozzle
    core_nozzle                       = SUAVE.Components.Energy.Converters.Expansion_Nozzle()   
    core_nozzle.tag                   = 'core nozzle'
    core_nozzle.polytropic_efficiency = 0.95
    core_nozzle.pressure_ratio        = 0.99    
    #add the core nozzle to the network    
    gt_engine.core_nozzle             = core_nozzle

    #Component 8 :fan nozzle
    fan_nozzle                       = SUAVE.Components.Energy.Converters.Expansion_Nozzle()   
    fan_nozzle.tag                   = 'fan nozzle'
    fan_nozzle.polytropic_efficiency = 0.95
    fan_nozzle.pressure_ratio        = 0.99
    #add the fan nozzle to the network
    gt_engine.fan_nozzle             = fan_nozzle

    #Component 9 : fan   
    fan                       = SUAVE.Components.Energy.Converters.Fan()   
    fan.tag                   = 'fan'
    fan.polytropic_efficiency = 0.93
    fan.pressure_ratio        = 1.7  
    #add the fan to the network
    gt_engine.fan             = fan    

    #Component 10 : thrust (to compute the thrust)
    thrust     = SUAVE.Components.Energy.Processes.Thrust()       
    thrust.tag ='compute_thrust'
    #total design thrust (includes all the engines)
    thrust.total_design   = 1392000.0* Units.N #Newtons

    #design sizing conditions
    altitude         = 35000.0*Units.ft
    mach_number      = 0.7

    # add thrust to the network
    gt_engine.thrust = thrust

    #size the turbofan
    turbofan_sizing(gt_engine,mach_number,altitude)   

    # add  gas turbine network gt_engine to the vehicle
    vehicle.append_component(gt_engine)      
    
    fuel                                  = SUAVE.Components.Physical_Component()
    vehicle.fuel                          = fuel
    fuel.mass_properties.mass             = vehicle.mass_properties.max_takeoff-vehicle.mass_properties.max_fuel
    fuel.origin                           = vehicle.wings.main_wing.mass_properties.center_of_gravity     
    fuel.mass_properties.center_of_gravity= vehicle.wings.main_wing.aerodynamic_center
    
    # ------------------------------------------------------------------
    #   Vehicle Definition Complete
    # ------------------------------------------------------------------

    return vehicle

# ----------------------------------------------------------------------
#   Define the Configurations
# ---------------------------------------------------------------------

def configs_setup(vehicle):
    
    # ------------------------------------------------------------------
    #   Initialize Configurations
    # ------------------------------------------------------------------

    configs = SUAVE.Components.Configs.Config.Container()

    base_config = SUAVE.Components.Configs.Config(vehicle)
    base_config.tag = 'base'
    configs.append(base_config)

    # ------------------------------------------------------------------
    #   Cruise Configuration
    # ------------------------------------------------------------------

    config = SUAVE.Components.Configs.Config(base_config)
    config.tag = 'cruise'

    configs.append(config)
    
    config.maximum_lift_coefficient = 1.2
    
    # ------------------------------------------------------------------
    #   Cruise with Spoilers Configuration
    # ------------------------------------------------------------------

    config = SUAVE.Components.Configs.Config(base_config)
    config.tag = 'cruise_spoilers'

    configs.append(config)
    
    config.maximum_lift_coefficient = 1.2


    # ------------------------------------------------------------------
    #   Takeoff Configuration
    # ------------------------------------------------------------------

    config = SUAVE.Components.Configs.Config(base_config)
    config.tag = 'takeoff'
    config.wings['main_wing'].control_surfaces.flap.deflection  = 20. * Units.deg
    config.wings['main_wing'].control_surfaces.slat.deflection  = 25. * Units.deg
    config.V2_VS_ratio = 1.21
    configs.append(config)
    
    # ------------------------------------------------------------------
    #   Landing Configuration
    # ------------------------------------------------------------------

    config = SUAVE.Components.Configs.Config(base_config)
    config.tag = 'landing'
    config.wings['main_wing'].control_surfaces.flap.deflection  = 30. * Units.deg
    config.wings['main_wing'].control_surfaces.slat.deflection  = 25. * Units.deg
    config.Vref_VS_ratio = 1.23
    configs.append(config)   
     
    # ------------------------------------------------------------------
    #   Short Field Takeoff Configuration
    # ------------------------------------------------------------------ 

    config = SUAVE.Components.Configs.Config(base_config)
    config.tag = 'short_field_takeoff'    
    config.wings['main_wing'].control_surfaces.flap.deflection  = 20. * Units.deg
    config.wings['main_wing'].control_surfaces.slat.deflection  = 25. * Units.deg
    config.V2_VS_ratio = 1.21
    
    configs.append(config)

    return configs
