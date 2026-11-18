Flight_Computer_Tested.ino was the original code used with the system.

Flight_Computer_Quaternion.ino will be the updated code for future flights --- missing BMP280 code but will be implemented before actual flight.

removed velocity calculation due to redundancy. FlightAnalysis.py will calculate velocity with trimmed flight_xxx_raw (named flight_xxx_trimmed) starting as flight starts

introduce new filtering to upgrade from complementary filter using quaternions. specifically a Mahony 6-axis IMU estimator
  - consistent attitude calculations when rotating on multiple axes
  - uses direction of gravity derived from acceleration readings to correct error in direction of gravity predicted from quaternions. integrates after error correction, which would lower error accumulated from integrating with original filter
  - easy to reapply to other flight systems, such as a drone
