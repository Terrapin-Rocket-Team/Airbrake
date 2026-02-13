# Airbrake Local Sim Assets

This folder contains local copies of simulation code used by
`Astra-Rocket/astra_support_sim.py`.

## Layout

- `flight_code_src/`: copied from `Airbrake/Flight_Code/src/`
  - `propagator.py`
  - `drag.py`
  - `shock.py`
  - `simulation.py`

## Why this exists

The copy keeps simulator iteration local to `Astra-Rocket`, so SITL integration
can evolve without changing legacy `Flight_Code`.

## Astra-Support source names

- `airbrake` (default, uses propagator model)
- `airbrake:propagator`
- `airbrake:csv:<dataset_name>` (legacy CSV playback)

