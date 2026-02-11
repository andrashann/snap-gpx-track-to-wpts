import argparse
import copy
import math
import sys

import gpxpy
import gpxpy.gpx


EARTH_RADIUS = 6_371_000  # meters


def haversine(lat1, lon1, lat2, lon2):
    """Distance in meters between two lat/lon points."""
    rlat1, rlon1, rlat2, rlon2 = (math.radians(v) for v in (lat1, lon1, lat2, lon2))
    dlat = rlat2 - rlat1
    dlon = rlon2 - rlon1
    a = math.sin(dlat / 2) ** 2 + math.cos(rlat1) * math.cos(rlat2) * math.sin(dlon / 2) ** 2
    return 2 * EARTH_RADIUS * math.asin(math.sqrt(a))


def project_onto_segment(wlat, wlon, alat, alon, blat, blon):
    """Project point W onto segment AB in lat/lon space.

    Returns (projected_lat, projected_lon, t) where t in [0,1] is the
    parameter along AB.  We work in a local equirectangular approximation
    which is accurate enough for short segments.
    """
    cos_lat = math.cos(math.radians((alat + blat) / 2))
    ax, ay = alon * cos_lat, alat
    bx, by = blon * cos_lat, blat
    wx, wy = wlon * cos_lat, wlat

    dx, dy = bx - ax, by - ay
    len_sq = dx * dx + dy * dy
    if len_sq == 0:
        return alat, alon, 0.0

    t = ((wx - ax) * dx + (wy - ay) * dy) / len_sq
    t = max(0.0, min(1.0, t))

    plat = alat + t * (blat - alat)
    plon = alon + t * (blon - alon)
    return plat, plon, t


def find_closest_on_segment(wpt, segment):
    """Find the closest point on a track segment to a waypoint.

    Returns (min_dist, seg_idx, projected_lat, projected_lon, t, vertex_idx).
    seg_idx is the index of the first point of the closest segment piece.
    t is the parameter along that segment piece [0,1].
    vertex_idx is the index of the closest existing vertex (or None if the
    closest point is between vertices).
    """
    points = segment.points
    if not points:
        return float("inf"), 0, 0, 0, 0, None

    best_dist = float("inf")
    best_seg_idx = 0
    best_plat = 0
    best_plon = 0
    best_t = 0

    for i in range(len(points) - 1):
        a, b = points[i], points[i + 1]
        plat, plon, t = project_onto_segment(
            wpt.latitude, wpt.longitude,
            a.latitude, a.longitude,
            b.latitude, b.longitude,
        )
        d = haversine(wpt.latitude, wpt.longitude, plat, plon)
        if d < best_dist:
            best_dist = d
            best_seg_idx = i
            best_plat = plat
            best_plon = plon
            best_t = t

    # Also check the last point on its own (for single-point segments or if
    # the last point is closest but wasn't the endpoint of a winning segment).
    if len(points) == 1:
        p = points[0]
        d = haversine(wpt.latitude, wpt.longitude, p.latitude, p.longitude)
        if d < best_dist:
            return d, 0, p.latitude, p.longitude, 0.0, 0

    # Determine if the projected point coincides with an existing vertex.
    TOLERANCE = 0.05  # meters
    vertex_idx = None
    if best_t < 1e-9:
        vertex_idx = best_seg_idx
    elif best_t > 1 - 1e-9:
        vertex_idx = best_seg_idx + 1
    else:
        # Check if the projected point happens to be very close to either end.
        d_a = haversine(best_plat, best_plon,
                        points[best_seg_idx].latitude, points[best_seg_idx].longitude)
        d_b = haversine(best_plat, best_plon,
                        points[best_seg_idx + 1].latitude, points[best_seg_idx + 1].longitude)
        if d_a < TOLERANCE:
            vertex_idx = best_seg_idx
        elif d_b < TOLERANCE:
            vertex_idx = best_seg_idx + 1

    return best_dist, best_seg_idx, best_plat, best_plon, best_t, vertex_idx


def find_closest_vertex(wpt, segment):
    """Find the closest existing track point (vertex) to a waypoint.

    Returns (min_dist, vertex_idx).
    """
    best_dist = float("inf")
    best_idx = 0
    for i, pt in enumerate(segment.points):
        d = haversine(wpt.latitude, wpt.longitude, pt.latitude, pt.longitude)
        if d < best_dist:
            best_dist = d
            best_idx = i
    return best_dist, best_idx


def make_bare_point(lat, lon):
    """Create a track point with only lat/lon (no elevation, time, etc.)."""
    return gpxpy.gpx.GPXTrackPoint(latitude=lat, longitude=lon)


def duplicate_point(pt):
    """Deep-copy an existing track point, preserving all attributes."""
    return copy.deepcopy(pt)


def process_add(gpx_data, maxdist):
    """Process all waypoints in 'add' mode."""
    waypoints = gpx_data.waypoints
    if not waypoints:
        return

    for track in gpx_data.tracks:
        for segment in track.segments:
            # Collect all insertions: (insert_index, [points_to_insert])
            insertions = []

            for wpt in waypoints:
                dist, seg_idx, plat, plon, t, vertex_idx = find_closest_on_segment(wpt, segment)

                if dist > maxdist:
                    continue

                if vertex_idx is not None:
                    # Closest point is an existing vertex — duplicate it and
                    # put the waypoint location between the original and dup.
                    orig = segment.points[vertex_idx]
                    dup = duplicate_point(orig)
                    wpt_pt = make_bare_point(wpt.latitude, wpt.longitude)
                    # Insert after the vertex: [orig] -> [orig, wpt_pt, dup]
                    insertions.append((vertex_idx + 1, [wpt_pt, dup]))
                else:
                    # Closest point is between two vertices — insert the
                    # projected point, then the waypoint, then a duplicate of
                    # the projected point.
                    proj = make_bare_point(plat, plon)
                    proj_dup = make_bare_point(plat, plon)
                    wpt_pt = make_bare_point(wpt.latitude, wpt.longitude)
                    # Insert after seg_idx: ... A [proj, wpt_pt, proj_dup] B ...
                    insertions.append((seg_idx + 1, [proj, wpt_pt, proj_dup]))

            # Apply insertions in reverse order of index to avoid shifting.
            insertions.sort(key=lambda x: x[0], reverse=True)
            for idx, pts in insertions:
                for p in reversed(pts):
                    segment.points.insert(idx, p)


def process_move(gpx_data, maxdist):
    """Process all waypoints in 'move' mode."""
    waypoints = gpx_data.waypoints
    if not waypoints:
        return

    for track in gpx_data.tracks:
        for segment in track.segments:
            for wpt in waypoints:
                dist, vidx = find_closest_vertex(wpt, segment)
                if dist > maxdist:
                    continue
                segment.points[vidx].latitude = wpt.latitude
                segment.points[vidx].longitude = wpt.longitude


def main():
    parser = argparse.ArgumentParser(
        description="Snap GPX tracks to waypoints.",
    )
    parser.add_argument("-i", required=True, help="Input GPX file")
    parser.add_argument("-d", type=float, default=100,
                        help="Maximum distance in meters (default: 100)")
    parser.add_argument("-m", choices=["add", "move"], default="add",
                        help="Mode: add (default) or move")
    parser.add_argument("-f", nargs="?", const="", default=None,
                        help="Output file. If omitted: stdout. "
                             "If given without a name: <infile>_snapped.gpx. "
                             "If given with a name: use that filename.")
    parser.add_argument("--overwrite", action="store_true",
                        help="Overwrite output file if it exists")
    args = parser.parse_args()

    with open(args.i, "r", encoding="utf-8") as f:
        gpx_data = gpxpy.parse(f)

    if not gpx_data.tracks:
        print("Warning: no tracks found in input file.", file=sys.stderr)
        return

    if not gpx_data.waypoints:
        print("Warning: no waypoints found in input file.", file=sys.stderr)
        return

    if args.m == "add":
        process_add(gpx_data, args.d)
    else:
        process_move(gpx_data, args.d)

    xml = gpx_data.to_xml()

    if args.f is None:
        sys.stdout.write(xml)
    else:
        import os
        if args.f == "":
            base, _ = os.path.splitext(args.i)
            dist = int(args.d) if args.d == int(args.d) else args.d
            outfile = f"{base}_snapped_{dist}.gpx"
        else:
            outfile = args.f
        if os.path.exists(outfile) and not args.overwrite:
            print(f"Error: {outfile} already exists. Use --overwrite to replace it.", file=sys.stderr)
            sys.exit(1)
        with open(outfile, "w", encoding="utf-8") as f:
            f.write(xml)
        print(f"Output written to {outfile}", file=sys.stderr)


if __name__ == "__main__":
    main()
