# Rover power and cumulative energy, published by ipex_energy/energy_meter.
#
# Energy counters only increase (drawn energy is positive), so the energy used
# over any window is: energy_j at the end - energy_j at the start.
# Counters restart at 0 when the meter starts or receives /ipex/energy/reset.

std_msgs/Header header

float64 elapsed_s           # seconds since the counters started

# Per channel (e.g. "drive/front_left", "rear_arm/drum")
string[] channels
float64[] power_w           # instantaneous power drawn [W]
float64[] energy_j          # cumulative energy drawn [J]
bool[] stale                # true if this channel's sensor data is missing / too old
                            # (power counted as 0 while stale)

# Per group: "drive", "rear_arm", "front_arm", "total"
string[] groups
float64[] group_power_w
float64[] group_energy_j
