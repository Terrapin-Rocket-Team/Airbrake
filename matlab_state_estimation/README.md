# Matlab State Estimation

This folder provides the tools to easily test various types of mock and 
real flight data against proposed state estimation filters. The entrance
point for this tool is main.m. Running main.m will do three things.

1. It will pull in or create mock flight data
1. Run the flight data through a filter
1. Plot the actual vs. measured vs. filter output

## Data Types

### Mock Data
This is entirely mock rocket data from just propogating equations of motion.
To set this up please edit the rocket parameters in section 1 under the Mock
data section.

### OpenRocket Data
This allows you to take an open rocket output data csv and use those values
to run through the state estimation. When exporting an open rocket sim csv
it is important to export all data (all columns) and to export in SI units
(m, kg, s).

### Flight Data
This is data that was logged by MMFS. Currently it supports data logged by 
MMFS v1.0.0 (aka 2023-24 Payload data).