# PhotonVision Config

Backup and inspection of PhotonVision coprocessor configurations for Theseus.

## Coprocessors

| Host | IP | Cameras |
|---|---|---|
| Coprocessor 13 | `10.49.82.13` | CameraBL, CameraFL |
| Coprocessor 14 | `10.49.82.14` | CameraBR, CameraFR |

SSH credentials: `photon` / `vision`

## Files

- `coprocessor-13_dump.json` — Pretty-printed config dump from `.13`
- `coprocessor-14_dump.json` — Pretty-printed config dump from `.14`
- `coprocessor-*/` — Raw SCP fetches (gitignored)
- `fetch_photon_config.py` — Fetch + dump script

## Usage

From `Theseus/`:

```bash
# Fetch config from both coprocessors and dump pretty JSON
python3 photon_config/fetch_photon_config.py

# Fetch from one coprocessor only
python3 photon_config/fetch_photon_config.py --host 13

# Re-dump existing SQLite without fetching
python3 photon_config/fetch_photon_config.py --dump-only
```

Requires `sshpass` (`brew install sshpass`).

## SQLite Schema

The PhotonVision database (`photon.sqlite`) has two tables:

**`cameras`** — one row per camera  
- `unique_name` (PK) — UUID
- `config_json` — nickname, device paths, quirks
- `drivermode_json` — driver mode settings
- `pipeline_jsons` — vision pipeline configs
- `otherpaths_json` — alternate device paths

**`global`** — key-value config  
- `hardwareConfig` — LED pins, GPIO commands
- `hardwareSettings` — LED brightness
- `networkConfig` — static IP, hostname, NT server address
- `neuralNetworkProperties` — neural net model paths and labels
