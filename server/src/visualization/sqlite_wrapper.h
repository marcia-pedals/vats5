#pragma once

#include <sqlite3.h>

#include <string>

namespace vats5::viz {

// RAII wrapper for sqlite3 database handle.
class SqliteDb {
 public:
  explicit SqliteDb(const std::string& path);
  ~SqliteDb();

  SqliteDb(const SqliteDb&) = delete;
  SqliteDb& operator=(const SqliteDb&) = delete;

  void exec(const char* sql);
  sqlite3* handle();

 private:
  sqlite3* db_ = nullptr;
};

// RAII wrapper for sqlite3 prepared statements.
class SqliteStmt {
 public:
  SqliteStmt(SqliteDb& db, const char* sql);
  ~SqliteStmt();

  SqliteStmt(const SqliteStmt&) = delete;
  SqliteStmt& operator=(const SqliteStmt&) = delete;

  void bind_text(int col, const char* val);
  void bind_double(int col, double val);
  void bind_int(int col, int val);
  void step_and_reset();

 private:
  sqlite3_stmt* stmt_ = nullptr;
};

}  // namespace vats5::viz
