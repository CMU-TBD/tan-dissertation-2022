

def get_most_recent_from_list(obj_list: list):

    latest_obj = None
    latest_time = None
    for obj in obj_list:
        if latest_time == None or obj.header.stamp > latest_time:
            latest_time = obj.header.stamp
            latest_obj = obj
    return latest_obj
