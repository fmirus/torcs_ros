Header header
# Virtual gas pedal (0 means no gas, 1 full gas)
float64 accel
# Virtual brake pedal (0 means no brake, 1 full brake)
float64 brake
# Virtual clutch pedal (0 means no clutch, 1 full clutch)
float64 clutch
# Gear value: -1,0,1,...,6 where -1 means backwards driving and 0 means idle
uint8 gear
# Steering value: -1 and +1 means respectively full right and
# left, that corresponds to an angle of 0.366519 rad
float64 steering
# Focus direction for the focus sensors in degrees. Currently unused
float64 focus
# This is meta-control command: 0 do nothing, 1 ask compe-
# tition server to restart the race. Currently unused
uint8 meta
