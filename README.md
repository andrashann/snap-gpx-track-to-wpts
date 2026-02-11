# snap-gpx-track-to-wpts

A small command-line Python tool that snaps tracks in GPX files to waypoints. This enables more precise distance calculations when waypoints are off-route.

## Installation

From PyPI (pip):

```
pip install snap-gpx-track-to-wpts
```

From PyPI (uv):

```
uv tool install snap-gpx-track-to-wpts
```

From source:

```
git clone https://github.com/andrashann/snap-gpx-track-to-wpts.git
cd snap-gpx-track-to-wpts
uv sync
```

## Usage

```
snap-gpx-track-to-wpts -i infile.gpx [-d maxdistance] [-m mode] [-f [outfile]]
```

When running from a source checkout, prefix with `uv run`:

```
uv run snap-gpx-track-to-wpts -i infile.gpx [-d maxdistance] [-m mode] [-f [outfile]]
```

| Argument | Required | Default | Description |
|----------|----------|---------|-------------|
| `-i infile.gpx` | Yes | | Input GPX file |
| `-d maxdistance` | No | `100` | Maximum distance (meters) from track to snap a waypoint; farther waypoints are discarded |
| `-m mode` | No | `add` | `add`: finds the point on the track closest to the waypoint -- not necessarily a track point itself, it could be between two track points e.g. on long straight segments. If it is not a track point, then we create this closest location as a track point. Then we create a duplicate, and between the original and the duplicate, we add the location of the waypoint as another track point. `move`: moves the closest track point to the waypoint location |
| `-f [outfile]` | No | stdout | If omitted: print to stdout. If `-f` with no argument: save to `<infile>_snapped_<maxdistance>.gpx`. If `-f outfile`: save to the specified file. |

## Notes

- If there are multiple tracks, all tracks are modified. For each track, all waypoints are taken into account.
- If there are no tracks in the file, exits with a warning but no error.
- If there are no waypoints in the file, exits with a warning but no error.
- Only the tracks are changed and only the relevant points. Attributes of track points, such as timestamp, elevation, or heart rate are not modified.
- Routes and waypoints are not modified.
