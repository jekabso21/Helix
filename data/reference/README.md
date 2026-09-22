# Reference data

`generic_2207_1900kv_5x4.3x3_6s_synthetic.csv` is **not a measurement**. It is generated from the
placeholder motor and prop constants in `configs/motors/generic_2207_1900kv.yaml` and
`configs/props/generic_5x4.3x3.yaml` (Kv 1900, DC-equivalent 0.18 ohm, 1.5 A no load,
k_t 1.5e-6, k_q 2.0e-8 at 25.2 V) so that `simctl propfit` has an input with the documented
columns. Replace it with a bench thrust table when one exists.

Columns: `throttle` (0..1), `voltage_v`, `rpm`, `current_a`, `thrust_g` or `thrust_n`, optional
`torque_nm`, optional `air_density_kg_m3` (default 1.225).
