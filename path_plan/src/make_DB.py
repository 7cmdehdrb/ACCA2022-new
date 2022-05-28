import sqlite3


class DB():
    def __init__(self):
        self.conn = sqlite3.connect(
            "/home/enbang/catkin_ws/src/ACCA2022-new/path_plan/path.db")
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

    # def search_all(self):
    #     self.cur.execute("SELECT * FROM PathPoint")
    #     for row in self.cur:
    #         print(row)


if __name__ == "__main__":

    db = DB()
    # db.createtable()
    point = [["A1", "B1", "A1B1"], ["B1", "C1", "B1C1"], ["C1", "D1", "C1D1"]]
    pathinfo = [['A1B1', 1, 0.1, 0.3, 0.1], [
        'A1B1', 2, 0.2, 0.8, 0.03], ['A1B1', 3, 0.1, 0.9, 0.08], ['B1C1', 1, 0.1, 0.3, 0.1], [
        'B1C1', 2, 0.2, 0.8, 0.03], ['B1C1', 3, 0.1, 0.9, 0.08]]
    for i in range(3, 6):
        db.pathinfo(pathinfo[i][0], pathinfo[i][1],
                    pathinfo[i][2], pathinfo[i][3], pathinfo[i][4])
    # db.cur.fetchall() all search

    db.conn.close()
