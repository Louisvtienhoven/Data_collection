import matplotlib
matplotlib.use('TkAgg')  # Add this before other imports
import pandas as pd
import matplotlib.pyplot as plt

# [Keep the rest of your original code]


# Load data with column names
columns = ['Channel_1', 'Channel_2', 'Channel_3',
           'Channel_4', 'Channel_5', 'Channel_6']
df = pd.read_csv(r'C:\Users\Louis\MSc-Thesis-Louis\2. Perform Measurement\Collected Data\100Hz_10.00min_2025-03-12_15-22-51.csv', header=None, names=columns)

# Create time axis based on sampling frequency
sampling_freq = 100  # Hz
duration_min = 10    # minutes
df['Time'] = df.index / sampling_freq  # Time in seconds

# Create figure with subplots
plt.figure(figsize=(15, 10))
plt.suptitle('Sensor Data Visualization (100 Hz Sampling)', y=1.02)

# Plot each channel with different colors
colors = ['#1f77b4', '#ff7f0e', '#2ca02c',
          '#d62728', '#9467bd', '#8c564b']

for i, col in enumerate(columns, 1):
    plt.subplot(3, 2, i)
    plt.plot(df['Time'], df[col], color=colors[i-1])
    plt.title(f'{col} vs Time')
    plt.xlabel('Time (s)')
    plt.ylabel('Amplitude')
    plt.grid(alpha=0.4)

plt.tight_layout()
plt.show()
