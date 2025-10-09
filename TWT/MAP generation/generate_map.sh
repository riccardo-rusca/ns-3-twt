#!/bin/bash

cd ..

# Generate the MCS-size map for the 2.4 GHz and 5 GHz bands, in uplink (--trafficType=1)
for mcs in $(seq 0 1 11); do
	for size in $(seq 50 50 65000); do
		echo "[2.4 GHz] Running simulation for buffer size (UDP payload): $size and mcs: $mcs..."
		./ns3 --run "twtUDP --simId=1 --frequency=2.4 --t0=17 --t1=17 --nStations=1 --twtDuration=100 --deadlines=1 --trafficType=1 --payloadSize=$size --mcs=$mcs --csv-output-file=mcs-size-map-2_4-new" > /dev/null 2>&1
	done
done

for mcs in $(seq 0 1 11); do
	for size in $(seq 50 50 65000); do
		echo "[5 GHz] Running simulation for buffer size (UDP payload): $size and mcs: $mcs..."
		./ns3 --run "twtUDP --simId=1 --frequency=5 --t0=17 --t1=17 --nStations=1 --twtDuration=100 --deadlines=1 --trafficType=1 --payloadSize=$size --mcs=$mcs --csv-output-file=mcs-size-map-5-new" > /dev/null 2>&1
	done
done