# Filtered GTFS Data

All filtered directories in `data/` are generated using `gtfs_filter_tool`,
which takes a single TOML config file as its argument. Config files are stored
in `data_configs/`.

The directory naming convention is:

```
RG_{YYYYMMDD}_{prefixes}
```

Where `{YYYYMMDD}` is the service day and `{prefixes}` are the trip ID prefix
filters (underscore-separated), or `all` for no prefix filter.

## Config file format

```toml
input_dir = "raw_RG_202506"
date = "20250718"
output_dir = "RG_20250718_BA"
prefixes = ["BA:"]     # optional; omit to include all trips
```

## 2025 directories

These were generated with date + prefix filtering **without** service day
combining (the tool now adds service day combining by default, which adds
additional `:prev-sd` and `:next-sd` trips).

```bash
gtfs_filter_tool data_configs/RG_20250718_BA.toml
gtfs_filter_tool data_configs/RG_20250718_CT.toml
gtfs_filter_tool data_configs/RG_20250718_BA_CT_SC.toml
```

Verified: trips and stop\_times match exactly. Routes, directions, and stops
match exactly for `RG_20250718_BA`; for `CT` and `BA_CT_SC` the current tool
produces a few extra stops/routes/directions from service day trips.

## 2026 directories

These were generated from `raw_RG_20251231` **with** service day combining
(the default behavior of `gtfs_filter_tool`).

```bash
gtfs_filter_tool data_configs/RG_20260108_all.toml
gtfs_filter_tool data_configs/RG_20260109_BA_CT_SC_SM_AC.toml
```
