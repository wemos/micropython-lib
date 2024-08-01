import sys
import ffilib
import uctypes


sq3 = ffilib.open("libsqlite3")

# int sqlite3_open(
#  const char *filename,   /* Database filename (UTF-8) */
#  sqlite3 **ppDb          /* OUT: SQLite db handle */
# );
sqlite3_open = sq3.func("i", "sqlite3_open", "sp")
# int sqlite3_config(int, ...);
sqlite3_config = sq3.func("i", "sqlite3_config", "ii")
# int sqlite3_close_v2(sqlite3*);
sqlite3_close = sq3.func("i", "sqlite3_close_v2", "p")
# int sqlite3_prepare(
#  sqlite3 *db,            /* Database handle */
#  const char *zSql,       /* SQL statement, UTF-8 encoded */
#  int nByte,              /* Maximum length of zSql in bytes. */
#  sqlite3_stmt **ppStmt,  /* OUT: Statement handle */
#  const char **pzTail     /* OUT: Pointer to unused portion of zSql */
# );
sqlite3_prepare = sq3.func("i", "sqlite3_prepare_v2", "psipp")
# int sqlite3_finalize(sqlite3_stmt *pStmt);
sqlite3_finalize = sq3.func("i", "sqlite3_finalize", "p")
# int sqlite3_step(sqlite3_stmt*);
sqlite3_step = sq3.func("i", "sqlite3_step", "p")
# int sqlite3_column_count(sqlite3_stmt *pStmt);
sqlite3_column_count = sq3.func("i", "sqlite3_column_count", "p")
# int sqlite3_column_type(sqlite3_stmt*, int iCol);
sqlite3_column_type = sq3.func("i", "sqlite3_column_type", "pi")
# int sqlite3_column_int(sqlite3_stmt*, int iCol);
sqlite3_column_int = sq3.func("i", "sqlite3_column_int", "pi")
# double sqlite3_column_double(sqlite3_stmt*, int iCol);
sqlite3_column_double = sq3.func("d", "sqlite3_column_double", "pi")
# const unsigned char *sqlite3_column_text(sqlite3_stmt*, int iCol);
sqlite3_column_text = sq3.func("s", "sqlite3_column_text", "pi")
# sqlite3_int64 sqlite3_last_insert_rowid(sqlite3*);
sqlite3_last_insert_rowid = sq3.func("l", "sqlite3_last_insert_rowid", "p")
# const char *sqlite3_errmsg(sqlite3*);
sqlite3_errmsg = sq3.func("s", "sqlite3_errmsg", "p")


SQLITE_OK = 0
SQLITE_ERROR = 1
SQLITE_BUSY = 5
SQLITE_MISUSE = 21
SQLITE_ROW = 100
SQLITE_DONE = 101

SQLITE_INTEGER = 1
SQLITE_FLOAT = 2
SQLITE_TEXT = 3
SQLITE_BLOB = 4
SQLITE_NULL = 5

SQLITE_CONFIG_URI = 17


class Error(Exception):
    pass


def check_error(db, s):
    if s != SQLITE_OK:
        raise Error(s, sqlite3_errmsg(db))


def get_ptr_size():
    return uctypes.sizeof({"ptr": (0 | uctypes.PTR, uctypes.PTR)})


class Connections:
    def __init__(self, h):
        self.h = h

    def cursor(self):
        return Cursor(self.h)

    def close(self):
        if self.h:
            s = sqlite3_close(self.h)
            check_error(self.h, s)
            self.h = None


class Cursor:
    def __init__(self, h):
        self.h = h
        self.stmnt = None

    def execute(self, sql, params=None):
        if self.stmnt:
            # If there is an existing statement, finalize that to free it
            res = sqlite3_finalize(self.stmnt)
            check_error(self.h, res)

        if params:
            params = [quote(v) for v in params]
            sql = sql % tuple(params)

        stmnt_ptr = bytes(get_ptr_size())
        res = sqlite3_prepare(self.h, sql, -1, stmnt_ptr, None)
        check_error(self.h, res)
        self.stmnt = int.from_bytes(stmnt_ptr, sys.byteorder)
        self.num_cols = sqlite3_column_count(self.stmnt)

        if not self.num_cols:
            v = self.fetchone()
            # If it's not select, actually execute it here
            # num_cols == 0 for statements which don't return data (=> modify it)
            assert v is None
            self.lastrowid = sqlite3_last_insert_rowid(self.h)

    def close(self):
        if self.stmnt:
            s = sqlite3_finalize(self.stmnt)
            check_error(self.h, s)
            self.stmnt = None

    def make_row(self):
        res = []
        for i in range(self.num_cols):
            t = sqlite3_column_type(self.stmnt, i)
            if t == SQLITE_INTEGER:
                res.append(sqlite3_column_int(self.stmnt, i))
            elif t == SQLITE_FLOAT:
                res.append(sqlite3_column_double(self.stmnt, i))
            elif t == SQLITE_TEXT:
                res.append(sqlite3_column_text(self.stmnt, i))
            else:
                raise NotImplementedError
        return tuple(res)

    def fetchone(self):
        res = sqlite3_step(self.stmnt)
        if res == SQLITE_DONE:
            return None
        if res == SQLITE_ROW:
            return self.make_row()
        check_error(self.h, res)


def connect(fname, uri=False):
    sqlite3_config(SQLITE_CONFIG_URI, int(uri))

    sqlite_ptr = bytes(get_ptr_size())
    sqlite3_open(fname, sqlite_ptr)
    return Connections(int.from_bytes(sqlite_ptr, sys.byteorder))


def quote(val):
    if isinstance(val, str):
        return "'%s'" % val
    return str(val)
