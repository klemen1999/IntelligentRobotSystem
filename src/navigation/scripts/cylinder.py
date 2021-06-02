import rospy

class Cylinder:

    def __init__(self, pose, color, seen_from, recommended_ring="white", model=None):
        self.pose = pose
        self.color = color
        self.seen_from = seen_from
        self.visited = False
        self.model = model
        # add classifier here instead of recommended_ring
        self.recommended_ring = recommended_ring

    def __str__(self):
        return f"\ncolor: {self.color}\n" \
               f"\nvisited: {self.visited}\n"


