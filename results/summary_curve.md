# curve track summary

| controller | mean_abs | max_abs | steady_state | iae | line_lost_frac |
| --- | --- | --- | --- | --- | --- |
| P | 0.01268 | 0.01626 | -0.01384 | 0.1023 | 0 |
| PI | 0.003695 | 0.01282 | 0.002232 | 0.02938 | 0 |
| PID | 0.003631 | 0.0123 | 0.001956 | 0.02881 | 0 |

Units: metres (errors) and metre-seconds (iae); arc rows only. `steady_state` is the signed mean of e_true over the final 40 % of the arc's duration. It is a tail-window measurement, not a converged equilibrium: the PI/PID integrator is still converging when the arc ends (see docs/RESULTS.md section 2).
