take raw data into excel. find where launch is detected. compare time to actual launch time. 

compare vertical acceleration to real life counterparts, vertical acceleration should be a large positive number at launch, ~-1g or ~-9.8m/s^2 once thrust is gone, until the rocket deploys parachute. 

vertical velocity calculated by Flight_Computer_Tested.ino, but new code, Flight_Computer_Quaternion.ino, will not calculate it due to redundancy. should remove to get rid of unneeded, unaccurate data. 

once trimmed run analysis and graph plotting on the new data labeled as flight_xxx_trimmed with FlightAnalysis.py
