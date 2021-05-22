from common_methods import color_name_from_rgba
import rospy


class Cylinder:

    def __init__(self, cylinder_id, location, color, seen_from, recommended_ring="white"):
        self.cylinder_id = cylinder_id
        self.location = location
        self.color = color
        self.seen_from = seen_from
        self.recommended_ring = recommended_ring

    def __str__(self):
        return f"location: {self.location}" \
               f"\ncolor: {self.color}\n" \
               f"recommended_ring: {self.recommended_ring}\n" \
               f"seen from: {self.seen_from}"

    def update_from_marker(self, marker):
        new_color = color_name_from_rgba(marker.color)
        if self.color != new_color:
            self.color = new_color
        self.location = marker.pose

