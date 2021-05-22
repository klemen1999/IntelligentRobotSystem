

class Task:

    def __init__(self, person, cylinder=None, ring=None):
        self.person = person
        self.cylinder = cylinder
        self.ring = ring

    def set_cylinder(self, cylinder):
        self.cylinder = cylinder

    def get_cylinder(self):
        return self.cylinder

    def set_ring(self, ring):
        self.ring = ring

    def get_ring(self):
        return self.ring

    def get_person(self):
        return self.person
