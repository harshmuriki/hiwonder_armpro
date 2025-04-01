class Database:
    def __init__(self):
        self.data = {274: 1, 49:1, 898:3, 100: 5, 714:5, 828: 7, 860: 7, 455: 7, 231: 9, 992:9, 904:9, 165:11}

    def box_id_to_bin_no(self, box_id):
        # Example mapping logic
        # In a real scenario, this could be a database lookup or a more complex algorithm
        if box_id in self.data:
            return self.data[box_id]
        else:
            return -1  # Return -1 if box_id is not found