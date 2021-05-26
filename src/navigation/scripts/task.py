class Task:

    def __init__(self, person=None, cylinder=None, ring=None):
        self.person = person
        self.cylinder = cylinder
        self.ring = ring

#
# class TaskList:
#     def __init__(self):
#         self.list = []
#
#     def addTask(self, newTask):
#         for task in self.list:
#             if task.person == newTask.person:
#                 return
#             if task.cylinder == newTask.cylinder:
#                 return
#             if task.ring == newTask.ring:
#                 return
#         self.list.append(newTask)

    # taski imajo vsi enako priorteto -> array taskov
    # če nemoreš naredit nobenga taska hodi naprej na map goalse

    # FACA= yes, cilinder=No
    # zapolni si da ko prideš do cilindra je naslednji task da greš to ring te barve
    # če ni detectan ta ring je to nov task
    # če je detectan pejdi takoj do njega in reši zadevo (vaccinate osebo)

    # Faca = yes, cilinder=yes
    # pejdi do ringa če je in rešu use
    # če ga ni dodaj u taks

    # Faca=no, cilinder=yes
    # usless, v cilindru je klasifikator, bomo vidli kdaj ga rabimo

    # face:no, cilinder=no
    # raziskuj naprej
