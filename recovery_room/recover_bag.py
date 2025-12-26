import sqlite3
import sys
import os

source_db = "wfc_lidar_0.db3"
target_db = "wfc_lidar_clean.db3"

if os.path.exists(target_db):
    os.remove(target_db)

print(f"Attempting to recover {source_db}...")

try:
    # Open source in Read-Only mode
    conn_src = sqlite3.connect(f"file:{source_db}?mode=ro", uri=True)
    conn_tgt = sqlite3.connect(target_db)
    
    curs_src = conn_src.cursor()
    curs_tgt = conn_tgt.cursor()

    # Get all tables
    curs_src.execute("SELECT name, sql FROM sqlite_master WHERE type='table';")
    tables = curs_src.fetchall()

    for table_name, create_sql in tables:
        print(f"Recovering table: {table_name}")
        
        # Create table in target
        curs_tgt.execute(create_sql)
        
        # Copy data row by row to skip bad blocks
        curs_src.execute(f"SELECT * FROM {table_name}")
        
        row_count = 0
        while True:
            try:
                rows = curs_src.fetchmany(1000)
                if not rows:
                    break
                
                # Dynamic placeholders based on column count
                placeholders = ','.join(['?'] * len(rows[0]))
                curs_tgt.executemany(f"INSERT INTO {table_name} VALUES ({placeholders})", rows)
                conn_tgt.commit()
                row_count += len(rows)
            except sqlite3.DatabaseError as e:
                print(f"  Skipping corrupted chunk in {table_name}: {e}")
                continue
                
        print(f"  - Recovered {row_count} rows.")

    print("Recovery Complete. Valid data saved to:", target_db)
    conn_src.close()
    conn_tgt.close()

except Exception as e:
    print("CRITICAL FAILURE:", e)
