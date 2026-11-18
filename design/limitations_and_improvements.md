limitations:
- as project kept developing, the total mass of the rocket quickly outpaced the power of the engines, resulting in a low apogee even with four engines total.
- four engines required the use of a cluster whip to launch with the launch system provided at the launch events.
- the length of wires made it difficult to add the barometer sensor to the avionics bay, especially when in a rush to launch, resulting in altitude estimation relying solely on integrated velocity, which is prone to a high accumulation of error.
- attitude estimations became confounded when gyroscope errors were integrated.

improvements:
- shave off weight by reducing filament thickness where possible
- move up to higher power engines
- cluster whip may be avoided if less engines are used. if less engines can be used is dependent on the thrust and impulse available from the higher power engines
- shorter or tied down wires could create more room in the avionics bay to house a barometer, which would give an additional, more accurate, estimation of altitude
- mahony filter, already implemented in Flight_Computer_Quaternion.ino would give more accurate pitch and roll values, which would better represent the rockets attitude during flight.
