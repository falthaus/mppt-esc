"""

Tool to simulate and validate floating point vs. fixed-point exponential
moving average (EMA)
see also https://en.wikipedia.org/wiki/Exponential_smoothing for the theory

Author: Felix Althaus


"""


import numpy as np
import matplotlib.pyplot as plt



# Filter configuration parameters
Q_EMA = 8       # Filter fractional bits
ALPHA = 0.30    # Smoothing factor of ideal EMA filer
N_MAG = 5.0     # Input signal noise level



# Internal/derived parameters
ALPHA_INT = round(ALPHA*(1<<Q_EMA))
T_STEP = 20



# Calculate and show time constant and response time
tau = -1 / np.log(1-ALPHA)
print()
print("Exponential Moving Average (EMA) Simulation")
print()
print(f"Smoothing factor:          {ALPHA:.3f} = {ALPHA_INT:d} (Q{Q_EMA:d})")
print(f"Time constant (tau):       {tau:.1f} cycles")
print(f"Response time (~5 x tau):  {5*tau:.1f} cycles")



# Caveat: Simulation only covers positive input values
# Input step signal (begin offset to avoid negative numbers, even with noise)
u = np.hstack([(2*N_MAG+1)*np.ones(T_STEP, dtype=np.uint32),
               100*np.ones(100, dtype=np.uint32)])
N = len(u)
n = np.random.uniform(-N_MAG, N_MAG, size=N)    # uniform noise (to bound min/max)
un = (np.round(u + n)).astype(np.uint32)        # step + noise
#
y = np.zeros(N)                                 # output buffer for ideal filer
y_int = np.zeros(N, dtype=np.uint32)            # output buffer for integer filter



# Initialize
y[0] = un[0]
y_int[0] = un[0]<<Q_EMA

# Simulate over all input data
for i in range(N-1):
    # Integer EMA with rescaling
    y_int[i+1] = (ALPHA_INT*(un[i]<<Q_EMA)) + ((1<<Q_EMA)-ALPHA_INT)*y_int[i]
    y_int[i+1] = int((y_int[i+1] +(1<<(Q_EMA-1)))>> Q_EMA)
    #
    # ideal (floating point) EMA
    y[i+1] =  ALPHA*un[i] + (1.0-ALPHA)*y[i]



# Plot figures
x = np.arange(N)
plt.figure()
plt.plot(x-T_STEP, u, 'kx-',
         x-T_STEP, un, 'x-',
         x-T_STEP, y, 'x-',
         x-T_STEP, (y_int +(1<<(Q_EMA-1)))>> Q_EMA, 'x-')
plt.legend(["Input (no noise)", "Input (with noise)",
            "Output (float EMA)", "Output (int EMA)"])

print()
print("Close figure to continue.")
plt.show()
