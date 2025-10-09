#!/bin/bash

# Compile the code of the C++ launcher
g++ -o launchercsv launchercsv.cpp mcs_size_map.h
chmod +x launchercsv

# Configure the project in optimized mode
cd ..
./ns3 configure --build-profile=optimized --enable-examples --disable-werror

# Pre-build the project
./ns3 build
cd 80211ax_sim_launcher

# This script is used to launch the journal simulations
numSTAs=16
tau=0.2
R=0.3
w=(0.1 0.3 0.5 0.7 0.9)
perc_tshold_val=1.6

# Launch the simulations for each value of "w"
for w_val in "${w[@]}"; do
  # Copy here the right file
  # cp "../TWT_FAB/Extension/NewScheduler-Results-05-07-2024/TWT_results_new_tau=0.2_R=0.3_w=${w_val}_STA=${numSTAs}.csv" input.csv
  cp "../TWT_FAB/Extension/fixedTWT_results/fixedTWT_results_new_tau=0.2_R=0.3_w=${w_val}_STA=${numSTAs}.csv" input.csv

  if [ $? -ne 0 ]; then
    # echo "File TWT_results_new_tau=0.2_R=0.3_w=${w_val}_STA=${numSTAs}.csv not found. Simulations stopped."
    echo "File fixedTWT_results_new_tau=0.2_R=0.3_w=${w_val}_STA=${numSTAs}.csv not found. Simulations stopped."
    exit
  fi

  ./launchercsv -f input.csv -m ${perc_tshold_val}
  rm input.csv
  mv ../outputTWT/output.csv "output_new_tau=0.2_R=0.3_w=${w_val}_STA=${numSTAs}_perc_${perc_tshold_val}.csv"

  # echo "Running simulations from file TWT_results_new_tau=0.2_R=0.3_w=${w_val}_STA=${numSTAs}_perc_${perc_tshold_val}.csv..."
  echo "Running simulations from file fixedTWT_results_new_tau=0.2_R=0.3_w=${w_val}_STA=${numSTAs}.csv..."
done