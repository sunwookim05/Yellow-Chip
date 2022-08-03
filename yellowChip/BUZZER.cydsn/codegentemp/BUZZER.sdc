# THIS FILE IS AUTOMATICALLY GENERATED
# Project: C:\Users\PC\Documents\PSoC Creator\yellowChip\BUZZER.cydsn\BUZZER.cyprj
# Date: Wed, 27 Jul 2022 07:36:38 GMT
#set_units -time ns
create_clock -name {mUART_SCBCLK(FFB)} -period 708.33333333333326 -waveform {0 354.166666666667} [list [get_pins {ClockBlock/ff_div_0}]]
create_clock -name {C12M(FFB)} -period 1000 -waveform {0 500} [list [get_pins {ClockBlock/ff_div_3}]]
create_clock -name {CyILO} -period 25000 -waveform {0 12500} [list [get_pins {ClockBlock/ilo}]]
create_clock -name {CyLFClk} -period 25000 -waveform {0 12500} [list [get_pins {ClockBlock/lfclk}]]
create_clock -name {CyIMO} -period 41.666666666666664 -waveform {0 20.8333333333333} [list [get_pins {ClockBlock/imo}]]
create_clock -name {CyHFClk} -period 41.666666666666664 -waveform {0 20.8333333333333} [list [get_pins {ClockBlock/hfclk}]]
create_clock -name {CySysClk} -period 41.666666666666664 -waveform {0 20.8333333333333} [list [get_pins {ClockBlock/sysclk}]]
create_generated_clock -name {mUART_SCBCLK} -source [get_pins {ClockBlock/hfclk}] -edges {1 17 35} -nominal_period 708.33333333333326 [list]
create_generated_clock -name {C12M} -source [get_pins {ClockBlock/hfclk}] -edges {1 25 49} [list]


# Component constraints for C:\Users\PC\Documents\PSoC Creator\yellowChip\BUZZER.cydsn\TopDesign\TopDesign.cysch
# Project: C:\Users\PC\Documents\PSoC Creator\yellowChip\BUZZER.cydsn\BUZZER.cyprj
# Date: Wed, 27 Jul 2022 07:36:37 GMT
