import numpy as np


GPS_POSITION_COLUMNS = slice(9, 12)
EARTH_RADIUS_METERS = 6378137.0


def gps_offsets_per_scan(matrix, scan_starts):
    """Interpolate per-scan GPS fixes into local east/north/up meter offsets."""
    offsets = np.zeros((len(scan_starts), 3), dtype=float)
    if not len(scan_starts) or matrix.ndim != 2 or matrix.shape[1] < 12:
        return offsets

    scan_ends = np.r_[scan_starts[1:], len(matrix)]
    scan_positions = np.full((len(scan_starts), 3), np.nan)
    scan_times = np.full(len(scan_starts), np.nan)
    for scan_index, (start, end) in enumerate(zip(scan_starts, scan_ends)):
        readings = matrix[start:end, GPS_POSITION_COLUMNS]
        valid_horizontal = np.isfinite(readings[:, :2]).all(axis=1)
        if valid_horizontal.any():
            scan_positions[scan_index, :2] = np.median(
                readings[valid_horizontal, :2], axis=0)

        valid_altitude = valid_horizontal & np.isfinite(readings[:, 2])
        if valid_altitude.any():
            scan_positions[scan_index, 2] = np.median(
                readings[valid_altitude, 2])
        if matrix.shape[1] > 12:
            timestamps = matrix[start:end, 8]
            if np.isfinite(timestamps).any():
                scan_times[scan_index] = np.median(
                    timestamps[np.isfinite(timestamps)])

    scan_indices = np.arange(len(scan_starts))
    valid_horizontal = np.isfinite(scan_positions[:, :2]).all(axis=1)
    if not valid_horizontal.any():
        return offsets

    fix_indices = np.flatnonzero(valid_horizontal)
    use_timestamps = (
        matrix.shape[1] > 12
        and np.isfinite(scan_times).all()
        and np.isfinite(matrix[scan_starts, 12]).any()
    )
    if use_timestamps:
        gps_timestamps = matrix[scan_starts, 12]
        timed_fixes = valid_horizontal & np.isfinite(gps_timestamps)
        if timed_fixes.any():
            timestamp_values, timestamp_groups = np.unique(
                gps_timestamps[timed_fixes], return_inverse=True)
            gps_positions = np.empty((len(timestamp_values), 3), dtype=float)
            timed_positions = scan_positions[timed_fixes]
            for group_index in range(len(timestamp_values)):
                gps_positions[group_index] = np.nanmedian(
                    timed_positions[timestamp_groups == group_index], axis=0)

            if len(timestamp_values) > 1:
                # Interpolate at each scan's IMU time, not at the slower GPS
                # update cadence, so repeated fixes don't create stationary clumps.
                gps_times = timestamp_values - timestamp_values[0]
                query_times = scan_times - timestamp_values[0]
                reference_latitude, reference_longitude = gps_positions[0, :2]
                east = (
                    EARTH_RADIUS_METERS * np.cos(np.radians(reference_latitude))
                    * np.radians(gps_positions[:, 1] - reference_longitude))
                north = (
                    EARTH_RADIUS_METERS
                    * np.radians(gps_positions[:, 0] - reference_latitude))
                offsets[:, 0] = np.interp(query_times, gps_times, east)
                offsets[:, 1] = np.interp(query_times, gps_times, north)

                valid_altitude = np.isfinite(gps_positions[:, 2])
                if valid_altitude.any():
                    altitude = np.interp(
                        query_times, gps_times[valid_altitude],
                        gps_positions[valid_altitude, 2])
                    offsets[:, 2] = (
                        altitude - gps_positions[np.flatnonzero(valid_altitude)[0], 2])
                return offsets

    for axis in range(2):
        scan_positions[:, axis] = np.interp(
            scan_indices, fix_indices, scan_positions[fix_indices, axis])

    reference_latitude, reference_longitude = scan_positions[fix_indices[0], :2]
    offsets[:, 0] = (
        EARTH_RADIUS_METERS * np.cos(np.radians(reference_latitude))
        * np.radians(scan_positions[:, 1] - reference_longitude))
    offsets[:, 1] = (
        EARTH_RADIUS_METERS
        * np.radians(scan_positions[:, 0] - reference_latitude))

    valid_altitude = np.isfinite(scan_positions[:, 2])
    if valid_altitude.any():
        altitude_indices = np.flatnonzero(valid_altitude)
        altitude = np.interp(
            scan_indices, altitude_indices,
            scan_positions[altitude_indices, 2])
        offsets[:, 2] = altitude - altitude[altitude_indices[0]]

    return offsets
