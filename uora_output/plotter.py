import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
import numpy as np
import os
from scipy.optimize import fsolve, root
import math
plt.rcParams.update({'font.size': 15})

# Parameters
W = 31  # Contention window min
Wmax = 127 # Contention windwo max
m = math.floor( math.log2(Wmax / W) )  # Maximum backoff stage
NRU = 9  # Total Resource Units (RUs) for 20MHz
E_P = 1700 * 8  # Expected packet size in bits

# Time durations (in microseconds)
T_BsrpTF = 0.124 * 1000
T_BasicTF = 0.136 * 1000
T_BSR = 0.0912 * 1000
T_BSR_ACK = 0.036 * 1000
T_P = 1.5168 * 1000
T_ACK = 0.036 * 1000
SIFS = 16
delta = 3

# Compute times based on equations
T1 = (T_BsrpTF + SIFS + delta) + (T_BSR + SIFS + delta) + (T_BSR_ACK + SIFS + delta) + (T_BasicTF + SIFS + delta) + (T_P + SIFS + delta) + (T_ACK + SIFS + delta)
T2 = (T_BsrpTF + SIFS + delta) + (T_P + SIFS + delta) + (T_ACK + SIFS + delta)
T3 = (T_BsrpTF + SIFS + delta) + (T_BSR + SIFS + delta)
T4 = (T_BsrpTF + SIFS + delta)

def tau_equation(p, N_RA):
    """Solve for tau given p using Equation (1)."""
    if N_RA <= 0 or (1 - 2 * p) == 0:  # Prevent division by zero
        return 0
    return (2 * (1 - 2*p)) / ((1 - 2 * p) * (W / N_RA + 1) + p * (W / N_RA) * (1 - (2 * p) ** m))

def p_equation(tau, N_RA, n_ra):
    """Solve for p given tau using Equation (3)."""
    if N_RA <= 0 or n_ra <= 0:
        return 0  # Avoid invalid exponentiation
    return 1 - (1 - tau / N_RA) ** (n_ra - 1)


# Define equations
def equations(vars, W, N_RA, n_ra):
    tau, p = vars
    if abs(1 - 2*p) < 1e-6:  # Prevent division by zero
        p += 1e-6
    #print(f"tau: {tau}, p: {p}")
    eq1 = tau - (2 * (1 - (2 * p))) / ((1 - 2 * p) * ((W / N_RA) + 1) + p * (W / N_RA) * (1 - (2 * p) ** m))
    eq2 = p - (1 - (1 - (tau / N_RA)) ** (n_ra - 1))
    return [eq1, eq2]


# Define total STAs range and different RU assignments
n_values = range(9, 100, 9)
N_RA_values = [0, 1, 3, 5, 7, 9]

# Store throughput results
throughput_values = []

# Define consistent colors for each N_RA value
colors = {
    0: 'blue', 
    1: 'green', 
    3: 'orange', 
    5: 'purple', 
    7: 'brown', 
    9: 'red'
}

# Compute throughput for different N_RA values
for N_RA in N_RA_values:
    throughput_values = []
    for n in n_values:
        NSA = NRU - N_RA # SA users 
        n_ra = n - NSA  # Compute n_ra, n stations contending for RUs

        if N_RA != 0:
            initial_guess = [0.4, 0.4]
            try:
                tau, p = fsolve(equations, initial_guess, args=(W, N_RA, n_ra), xtol=1e-6, full_output=False)
            except Exception as e:
                print(f"Solver failed at N_RA={N_RA}, n={n}: {e}")
                tau, p = 0, 0  # Assign default values when solver fails
                
            p2 = p_equation(tau, N_RA, n_ra);
            
            # Compute probabilities
            P_tr = 1 - (1 - (tau / N_RA)) ** n_ra
            P_s = (n_ra * (tau / N_RA) * (1 - (tau / N_RA)) ** (n_ra - 1)) / (1 - (1 - (tau / N_RA)) ** n_ra)
            P_idle = (1 - P_tr) ** N_RA
            P_1 = 1 - (1 - P_tr * P_s) ** N_RA

        # Compute throughput S
        if NSA == NRU:  # All RUs assigned to SA
            S = (NSA * E_P) / T1
        elif 1 <= NSA < NRU:  # Mixed SA and RA
            S = ( (NSA + (N_RA * P_tr * P_s) ) * E_P) / T1
        else:  # All RUs assigned to RA
            #print (f"{P_s} \t {tau} \t{P_tr}")
            S = (N_RA * P_tr * P_s * E_P) / ( (P_1 * T1) + (P_idle * T4) + (1 - P_1 - P_idle) * T3)

        throughput_values.append(S)

    # Plot the throughput for this N_RA setting
    plt.plot(n_values, throughput_values, marker='o', linewidth=2, markersize=5, color=colors[N_RA], label=f'N_RA = {N_RA}')

# Iterate over different RARU values
for rarus in [0] + [z for z in range(1, 10, 2)]:
    dplt = pd.DataFrame()
    
    # Collect data from different runs and STAs
    for runs in range(1, 6):
        for nSta in [z for z in range(9, 101, 9)]:
            file_path = f"uora_validation/{runs}-{nSta}-{rarus}/throughput.out"
            
            if os.path.exists(file_path):
                df_ap = pd.read_csv(file_path, delimiter="\t")
                if df_ap.dropna(how='all').empty:
                    print(f"No data in {file_path}")
                    continue
            else:
                print(f"No file for {file_path}")
                continue
            
            # Drop irrelevant columns
            df_ap.drop(columns=["MCS", "Bandwidth_MHz", "Guard_interval_ns"], inplace=True, axis=1)
            # Add nSta as a label
            df_ap["nSta"] = pd.Series([nSta for _ in range(len(df_ap.index))])
            # Concatenate the data
            dplt = pd.concat([dplt, df_ap], ignore_index=True)
    
    # Calculate the 99th percentile throughput
    df_mean = dplt.groupby('nSta', as_index=False).quantile(q=0.99)
    
    # Plot the results
    plt.plot(
        df_mean["nSta"], 
        df_mean["Throughput_Mbps"], 
        linestyle='dashed', 
        linewidth=2,
        marker='s', 
        markersize=5,
        color=colors[rarus]
        #label=f"{rarus} - RARUs"
    )

plt.xlabel("Number of STAs")
plt.ylabel("Throughput (Mbps)")
plt.legend()
plt.grid()
#plt.show()
plt.savefig(f"mmodel_and_sim.png", dpi=500)
