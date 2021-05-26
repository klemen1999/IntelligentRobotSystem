
class Person:

    def __init__(self, pose, cylinder="white", age=0, mask=False, training=False, visited=False, point=None, vaccinated=False):
        self.age = age
        self.pose = pose
        self.cylinder = cylinder
        self.mask = mask
        self.training = training
        self.visited = visited
        self.approachPoint = point
        self.vaccinated = vaccinated

    def __str__(self):
        return f"age: {self.age} \ncylinder: {self.cylinder} " \
               f"\nmask: {self.mask}  \ntraining: {self.training} \nvisited: {self.visited} " \
               f"\nvacc: {self.vaccinated}"
