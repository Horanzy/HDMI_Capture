open_project test.gprj
set_option -top_module it68051_capture_top
set_option -use_sspi_as_gpio 1
set_option -use_i2c_as_gpio 1
set_option -use_cpu_as_gpio 1
set_option -gen_text_timing_rpt 1
set_option -timing_driven 1
# Pin the selected P&R strategy; otherwise Gowin can reuse the previous run's
# place/route options from local project state.
set_option -place_option 0
set_option -route_option 0
set_option -correct_hold_violation 1
run syn
run pnr
run close
exit
