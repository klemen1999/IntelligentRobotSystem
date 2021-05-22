
class Person:

    def __init__(self, person_id, location, cylinder="white", age=0, mask=False):
        self.person_id = person_id
        self.age = age
        self.location = location
        self.cylinder = cylinder
        self.mask = mask

    def __str__(self):
        return f"age: {self.age} \nlocation: {self.location} \ncylinder: {self.cylinder} \nmask: {self.mask}"

    def set_age(self, age):
        print(f"Setting age for person {self.person_id} to {age}")
        self.age = age

    def get_age(self):
        return self.age

    def update_from_marker(self, marker):
        self.location = marker.pose
