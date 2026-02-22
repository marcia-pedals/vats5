# Read the SQL file, substitute into template, and write the header.
file(READ "${SQL_IN}" VIZ_SCHEMA_SQL)
configure_file("${TEMPLATE_IN}" "${HEADER_OUT}")
