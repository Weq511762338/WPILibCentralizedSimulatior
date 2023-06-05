# WPILibCentralizedSimulatior

# Structure

- containing actual motors (CANSparkMax, TalonFX). The motors will be accessed through this wrapper. All functions will be inherited by this


# Strategy

- This wrapper will only be turned on or be "used" in simulation, not when running on real robot. So it is fine for it to use generic Encoder and example stats for devices