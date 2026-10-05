# import

The sim reads one game-free dataset: a `Car` (`sim/src/car.zig`) and a list of `Material`
(`sim/src/material.zig`), as ZON files. The sim holds no values from a game. It has made-up
cars and surfaces in `sim/data/`.

If you own a game, these scripts translate its files into the dataset. Each script is one file.
It uses the Python standard library only.

| Script | Game | Writes |
| --- | --- | --- |
| `d3.py` | DiRT 3 | `<out>/d3-<car>/car.zon`, `<out>/d3/materials.zon` |
| `fh1.py` | Forza Horizon 1 | `<out>/fh-<car>/car.zon`, `<out>/fh/materials.zon` |

DiRT Rally 1 has no import script. Its car file comes from a memory snapshot (`tools/dr1/specdiff.py`).

## DiRT 3

1. Set `D3` to the game folder, or give `--install DIR`.
2. Run `python3 import/d3.py ffr`. Give more car names (folders in `cars/models`) to import more cars.
3. The files go to `runs/`. Give `--out DIR` for a different folder.

The script reads the car's `.ctf`, its `.nd2` (wheels and hull), its `_highLOD.pssg` (the drawn
wheel width), `cars/settings/tuning.tng` (the setup sliders), `surface_materials.xml` and
`dirt3_game.exe` (the substep lengths). It follows the game's own float order, so the values
have the game's bits. `--abs-assist` gives the ABS of the game's assist setting.
`--final-drive 0..1` sets the simple-setup final drive slider. The default is a new profile's
(the slider at its top), which the D3 checks use.

## Forza Horizon 1

See the usage at the top of `fh1.py`. The car database is `gamedb.slt`. The surfaces come from
`physics.zip` (`surfaceTypes.xml`). The script uses `tools/fh/fhzip.py` to read the Xbox zips.

## Use the files

- The demo lists each car that it finds in `runs/` (`d3-*`, `dr1-*`, `fh-*`).
- `sim_load_d3_car`, `sim_load_dr1_car` and `sim_load_fh_car` take a car file and a list of materials.
- `sim_load_track` takes a D3 `track.jpk` and `runs/d3/materials.zon`.
- The D3 checks in `tools/` read `runs/d3-ffr/car.zon` and `runs/d3/materials.zon` (`tools/simlib.py`).
