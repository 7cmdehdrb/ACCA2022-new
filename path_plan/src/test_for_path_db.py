import sqlite3
import rospkg
from DB import *

# db_path = rospkg.RosPack().get_path("path_plan") + "/path.db" # 절대 경로
db = DB()
db_path = '/home/jinyoung/database_file_4_test.db'
conn = sqlite3.connect(db_path)
cur = conn.cursor()
print(db.checkDB()[1])
'''cur.execute("SELECT * FROM PathPoint")
# cur.execute("SELECT * FROM PathInfo")

table = cur.fetchall()[0][0]
print(type(table))  # -> list
print(db_path)  # -> 절대 경로

for i in table:
    print(i)'''
