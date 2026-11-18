#data processor and analysis for .csv file derived from original sensor data.

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

df = pd.read_csv('flight_001_trimmed.csv')

from scipy.integrate import cumulative_trapezoid

#calculate velocity
from scipy.integrate import cumulative_trapezoid
time = df['FlightTime_s'].to_numpy()
accel = df['FilteredVerticalAccel_ms2'].to_numpy()
velocity = cumulative_trapezoid(accel, time, initial=0)
df['VerticalVelocity_ms'] = velocity

#calculate roll
df['Roll_Relative_deg'] = (df['Roll_deg'] - df['Roll_deg'].iloc[0])

#calculate pitch
df['Pitch_Relative_deg'] = (df['Pitch_deg'] - df['Pitch_deg'].iloc[0] - 5.00000)

#update file
df.to_csv('flight_001_processed.csv', index=False)

print(df.head())
print(df.info())
print(df.describe())

#vertical acceleration
plt.figure(figsize=(12,5))
plt.scatter(df['FlightTime_s'],
         df['FilteredVerticalAccel_ms2'],
         s=15)
plt.axhline(0, color='black', linewidth=1.5)
plt.xlabel('Time after launch detection (s)')
plt.ylabel('Filtered vertical acceleration (m/s^2)')
plt.title('Filtered vertical acceleration recorded during flight')
plt.grid()
plt.show()

#vertical velocity
plt.figure(figsize=(12,5))
plt.scatter(df['FlightTime_s'],
         df['VerticalVelocity_ms'],
         s=15)
plt.axhline(0, color='black', linewidth=1.5)
plt.xlabel('Time after launch detection (s)')
plt.ylabel('Integrated velocity (m/s)')
plt.title('Integrated velocity estimate during flight')
plt.grid()
plt.show()

#roll and pitch
plt.figure(figsize=(12,5))
plt.scatter(df['FlightTime_s'], df['Roll_Relative_deg'], label= 'Roll', s=15)
plt.scatter(df['FlightTime_s'], df['Pitch_Relative_deg'], label= 'Pitch', s=15)
plt.axhline(0, color='black', linewidth=1.5)
plt.xlabel('Time after launch detection (s)')
plt.ylabel('Calculated Angle (degrees)')
plt.title('Gyroscope-based roll and pitch estimates during flight')
plt.legend(loc='upper right')
plt.grid()
plt.show()