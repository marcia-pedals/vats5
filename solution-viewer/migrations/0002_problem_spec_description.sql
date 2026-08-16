-- A prose description of what a problem spec actually asks for, beyond what
-- its short title fits. Unlike `title` it is not an identifier of any kind, so
-- it has no uniqueness or length constraint.
--
-- The default is only here to fill in the rows that already exist; every spec
-- config carries a description, so the next sync_configs run overwrites them.
-- Dropped afterwards so that a later insert has to supply one.
ALTER TABLE problem_spec ADD COLUMN description TEXT NOT NULL DEFAULT '';
ALTER TABLE problem_spec ALTER COLUMN description DROP DEFAULT;
