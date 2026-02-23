#include "visualization/sqlite_wrapper.h"

#include <stdexcept>

namespace vats5::viz {

SqliteDb::SqliteDb(const std::string& path) {
  int rc = sqlite3_open(path.c_str(), &db_);
  if (rc != SQLITE_OK) {
    std::string err = sqlite3_errmsg(db_);
    sqlite3_close(db_);
    db_ = nullptr;
    throw std::runtime_error("Failed to open SQLite database: " + err);
  }
}

SqliteDb::~SqliteDb() {
  if (db_) sqlite3_close(db_);
}

void SqliteDb::exec(const char* sql) {
  char* err_msg = nullptr;
  int rc = sqlite3_exec(db_, sql, nullptr, nullptr, &err_msg);
  if (rc != SQLITE_OK) {
    std::string err = err_msg;
    sqlite3_free(err_msg);
    throw std::runtime_error("SQLite exec error: " + err);
  }
}

sqlite3* SqliteDb::handle() { return db_; }

SqliteStmt::SqliteStmt(SqliteDb& db, const char* sql) {
  int rc = sqlite3_prepare_v2(db.handle(), sql, -1, &stmt_, nullptr);
  if (rc != SQLITE_OK) {
    throw std::runtime_error(
        std::string("Failed to prepare statement: ") +
        sqlite3_errmsg(db.handle())
    );
  }
}

SqliteStmt::~SqliteStmt() {
  if (stmt_) sqlite3_finalize(stmt_);
}

void SqliteStmt::bind_text(int col, const char* val) {
  sqlite3_bind_text(stmt_, col, val, -1, SQLITE_TRANSIENT);
}

void SqliteStmt::bind_double(int col, double val) {
  sqlite3_bind_double(stmt_, col, val);
}

void SqliteStmt::bind_int(int col, int val) {
  sqlite3_bind_int(stmt_, col, val);
}

void SqliteStmt::step_and_reset() {
  sqlite3_step(stmt_);
  sqlite3_reset(stmt_);
}

}  // namespace vats5::viz
