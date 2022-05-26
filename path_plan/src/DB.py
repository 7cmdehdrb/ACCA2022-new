import sqlite3


class DB():
    def __init__(self):
        self.conn = sqlite3.connect(
            "/home/enbang/catkin_ws/src/ACCA2022-new/path.db")
        self.cur = self.conn.cursor()

    def createtable(self):
        self.cur.execute(
            "CREATE TABLE PathPoint(Start_point CHAR, End_point CHAR, path_id CHAR key unique);")
        self.cur.execute(
            "CREATE TABLE PathInfo(path_id CHAR,idx INT, x FLOAT, y FlOAT, yaw FLOAT);")

    def createpoint(self, start, end, path_id):
        query = "INSERT INTO PathPoint VALUES(?, ?, ?);"
        self.cur.execute(query, (start, end, path_id))
        self.conn.commit()

    def pathinfo(self, path_id, idx, x, y, yaw):
        query = "INSERT INTO PathInfo VALUES(?, ?, ?, ?, ?);"
        self.cur.execute(query, (path_id, idx, x, y, yaw))
        self.conn.commit()

    def search_all(self):
        self.cur.execute("SELECT * FROM PathPoint")
        for row in self.cur:
            print(row)


if __name__ == "__main__":

    db = DB()
    db.createtable()
    db.createpoint("A1", "B1", "A1B1")
    db.pathinfo('A1B1', 1, 0.1, 0.3, 0.1)
    # db.search_all()
    db.conn.close()
