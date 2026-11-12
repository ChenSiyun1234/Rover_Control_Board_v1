# 24 V robot-axis hardware study

`DESIGN.md` contains requirements, source links, hand calculations, modelling assumptions and qualification limits. `RESULTS.md` is generated from ngspice, with full corner data in each circuit folder. Schematics and waveforms are in `figures/`.

Run from the repository root:

```powershell
python hw/run_all.py
python -m pytest -q
```

ngspice 46 is required as a shared library (KiCad 10 ships one). Set
`NGSPICE_DLL` to its library path, or use the default KiCad 10 installation
under LOCALAPPDATA/Programs or ProgramFiles. Inaccessible candidates are skipped.
The wrapper drives ngspice synchronously through ctypes using built-in devices.
Do not run it concurrently in threads: ngspice has process-global state.
`tests/test_hw.py` takes about 40 s.

Independent circuit runners are also available as `python -m hw.io24.run` and `python -m hw.brake.run`. `run_all.py` is the authoritative artifact regeneration command. Pytest invokes fresh simulations and never reads the CSV/report values as test inputs. It creates no large temporary simulation files. No janitor marker is required because no basetemp lifecycle is used by these tests.

Files:

- `spice.py`: shared-library initialization, circuit execution, vector extraction and copying.
- `io24/run.py`, `input.cir`, `step.cir`: input circuit generator and nominal netlists.
- `io24/corners.csv`, `dc.csv`, `out/step_ctr1.csv`, `out/step_ctr4.csv`: generated input results.
- `brake/run.py`, `clamp_a.cir`, `clamp_b.cir`, `clamp_c.cir`, `reset.cir`: brake circuit generator and nominal/reset netlists.
- `brake/corners.csv`, `out/waveform_a.csv`, `out/waveform_b.csv`, `out/waveform_c.csv`, `brake/reset.csv`: generated brake results.
- `draw.py`: Schemdraw schematic generation.
- `figures/io24_schematic.svg`, `brake_a_schematic.svg`, `brake_b_schematic.svg`, `brake_c_schematic.svg`, plus PNG previews.
- `figures/input_dc.png`, `input_step.png`, `brake_turnoff.png`: simulated waveforms.
- `summary.json`: generated machine-readable summary.
- `../tests/test_hw.py`, `../pytest.ini`: fresh-simulation requirement checks and test configuration.

This is a design/verification study. The meaning of a simulated guarantee and the remaining datasheet qualification gaps are stated explicitly in DESIGN and RESULTS.
