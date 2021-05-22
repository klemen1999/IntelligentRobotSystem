from common_methods import color_name_from_rgba


class Ring:

    def __init__(self, ring_id, location, color):
        self.ring_id = ring_id
        self.color = color
        self.location = location

    def __str__(self):
        return f"color: {self.color} \n location: {self.location}"

    def update_from_marker(self, marker):
        new_color = color_name_from_rgba(marker.color)
        if self.color != new_color:
            self.color = new_color
        self.location = marker.pose
