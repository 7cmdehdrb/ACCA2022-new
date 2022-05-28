import sqlite3


class Path():
    def __init__(self):
        self.conn = sqlite3.connect(
            "/home/enbang/catkin_ws/src/ACCA2022-new/path_plan/path.db")
        self.cur = self.conn.cursor()

    def bring_path_id(self, s_point, e_point):
        query = "SELECT path_id From PathPoint Where Start_point = '%s' AND End_point = '%s';"
        path_id = self.cur.execute(
            query % (s_point, e_point)).fetchall()[0][0]
        path_id = str(path_id)
        return path_id

    def bring_pathinfo(self, path_id):
        query = "SELECT x, y, yaw From PathInfo  Where path_id = '%s';"
        pathinfo = self.cur.execute(query % (path_id)).fetchall()
        return pathinfo


# main
path = Path()
id = [['A1', 'B1'], ['B1', 'C1']]
for i in range(len(id)):
    path_id = path.bring_path_id(id[i][0], id[i][1])
    pathinfo = path.bring_pathinfo(path_id)
    print(i)
    print(path_id)
    print(pathinfo)
