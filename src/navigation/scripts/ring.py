
class Ring:

    def __init__(self, pose, color):
        self.color = color
        self.pose = pose

    def __str__(self):
        return f"color: {self.color} \n location: {self.pose}"
