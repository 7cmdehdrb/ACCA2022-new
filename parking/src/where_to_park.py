import numpy as np


# 1순위 : 3개이상 연속 또는 2개 연속이지만 끝 자리
# 2순위 : 2개 이상 연속 또는 연속자리는 아니지만 끝자리
# 3순위 : 1순위, 2순위 모두에 해당하지 않는 자리

available_zone = []
the_number_of_parkinarea = 6


def where_to_park(available_zone):
    available_zone.insert(0, 0)
    available_zone.append(the_number_of_parkinarea)
    row = []
    area_in_a_row = []
    temp_of_area_in_a_row = []
    j = -3
    k = 0
    number_of_in_a_row = 0

    for i in available_zone:
        if i != j + 1:
            row.append(number_of_in_a_row)
            area_in_a_row.append(temp_of_area_in_a_row)
            number_of_in_a_row = 0
            temp_of_area_in_a_row = []
            if len(area_in_a_row) < len(row):
                area_in_a_row.append([])
                k += 1
        else:
            number_of_in_a_row += 1
            temp_of_area_in_a_row.append(i)

    Max = max(row)
    loc_of_Max = row.index(Max)
    candidate_area = area_in_a_row[loc_of_Max]
    result = candidate_area[int(len(candidate_area)/2)+1]

    return result
