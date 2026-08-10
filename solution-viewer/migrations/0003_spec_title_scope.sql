-- problem_spec titles are now scoped to their target stop set (the UI shows
-- them as columns under a target_stops heading), so they only have to be
-- unique within one set rather than across the whole source.

ALTER TABLE problem_spec
  DROP CONSTRAINT problem_spec_gtfs_source_id_title_key;

ALTER TABLE problem_spec
  ADD CONSTRAINT problem_spec_gtfs_source_id_target_stops_id_title_key
  UNIQUE (gtfs_source_id, target_stops_id, title);
