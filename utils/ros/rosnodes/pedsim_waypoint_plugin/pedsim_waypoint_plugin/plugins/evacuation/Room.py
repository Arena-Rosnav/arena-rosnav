import numpy as np

'''class which defines the room the simulation runs in.
The room consists of n walls. Each wall is is a line 
and is represented in the code by the 
tupel of its endpoints.'''

class Room:
    def __init__(self, room, room_size):
        self.room_size = room_size
        if room == "square":
            self.wallshere = False
            self.door_size = room_size/15                    # size of the door is proportional to the size of the room
            self.destination = np.array([[-0.5, room_size/2]])   # destination the agenst want to go
            self.num_walls = 5
            self.walls = np.array([[[0, 0], [0, room_size/2-self.door_size/2]], # wall 1
                          [[0, room_size/2+self.door_size/2], [0, room_size]],  # wall 2
                          [[0, room_size], [room_size, room_size]],             # wall 3
                          [[room_size, room_size], [room_size, 0]],             # wall 4
                          [[room_size, 0], [0, 0]]])                            # wall 5
            # agents spawn with x and y position between 1 and (room_size-1) 
            self.spawn_zone = np.array([[room_size/2, room_size-1], [1, room_size-1]])

        if room == "long_room":
            self.wallshere = False
            self.door_size = room_size/15                    # size of the door is proportional to the size of the room
            self.room_len = room_size
            self.room_with = room_size/5
            self.destination = np.array([[-0.5, self.room_with/2]])   # destination the agenst want to go
            self.num_walls = 5
            self.walls = np.array([[[0, 0], [0, self.room_with/2-self.door_size/2]],        # wall 1
                          [[0, self.room_with/2+self.door_size/2], [0, self.room_with]],    # wall 2
                          [[0, self.room_with], [self.room_len, self.room_with]],           # wall 3
                          [[self.room_len, self.room_with], [self.room_len, 0]],            # wall 4
                          [[self.room_len, 0], [0, 0]]])                                    # wall 5
            # agents spawn with x and y position between 1 and (room_size-1) 
            self.spawn_zone = np.array([[1, self.room_len-1], [1, self.room_with-1]])

        if room == "long_room_v2":
            self.wallshere = False
            self.door_size = room_size/15                    # size of the door is proportional to the size of the room
            self.room_len = room_size
            self.room_with = room_size/5
            self.destination = np.array([[-0.5, self.room_with/2], [self.room_len + 0.5, self.room_with/2]])   # destination the agenst want to go
            self.num_walls = 6
            self.walls = np.array([[[0, 0], [0, self.room_with/2-self.door_size/2]], # wall 1
                          [[0, self.room_with/2+self.door_size/2], [0, self.room_with]],  # wall 2
                          [[0, self.room_with], [self.room_len, self.room_with]],             # wall 3
                          [[self.room_len, self.room_with], [self.room_len, self.room_with/2+self.door_size/2]],
                          [[self.room_len, self.room_with/2-self.door_size/2], [self.room_len, 0]],                 # wall 4
                          [[self.room_len, 0], [0, 0]]])                            # wall 5
            # agents spawn with x and y position between 1 and (room_size-1)
            self.spawn_zone = np.array([[1, self.room_len-1], [1, self.room_with-1]])

        if room == "edu_1":
            self.wallshere = True
            self.door_size = room_size/15                    # size of the door is proportional to the size of the room
            self.destination = np.array([[-0.5, room_size/2]])   # destination the agenst want to go
            self.num_walls = 6
            self.walls = np.array([[[0, 0], [0, room_size/2-self.door_size/2]], # wall 1
                          [[0, room_size/2+self.door_size/2], [0, room_size]],  # wall 2
                          [[0, room_size], [room_size, room_size]],             # wall 3
                          [[room_size, room_size], [room_size, 0]],             # wall 4
                          [[room_size, 0], [0, 0]],                             # wall 5
                          [[room_size/4, room_size*0.3], [room_size/4, room_size*0.7]]])           # wall 6      
            # agents spawn with x and y position between 1 and (room_size-1) 
            self.spawn_zone = np.array([[room_size/2, room_size-1], [1, room_size-1]])

        if room == "edu_room":
            self.wallshere = True
            self.door_size = room_size/15                    # size of the door is proportional to the size of the room
            self.destination = np.array([[-0.5, room_size/2]])   # destination the agenst want to go
            self.num_walls = 7
            self.walls = np.array([[[0, 0], [0, room_size/2-self.door_size/2]], # wall 1
                          [[0, room_size/2+self.door_size/2], [0, room_size]],  # wall 2
                          [[0, room_size], [room_size, room_size]],             # wall 3
                          [[room_size, room_size], [room_size, 0]],             # wall 4
                          [[room_size, 0], [0, 0]],                             # wall 5
                          [[9/25*room_size, room_size*0.15], [5.5/25*room_size, room_size*0.4]],            # wall 6
                          [[5.5/25*room_size, room_size*0.6], [9/25*room_size, room_size*0.85]]])           # wall 7                 
            # agents spawn with x and y position between 1 and (room_size-1) 
            self.spawn_zone = np.array([[room_size/2, room_size-1], [1, room_size-1]])



    def get_wall(self, n):              # gives back the endpoints of the nth wall
        return self.walls[n,:,:]

    def get_num_walls(self):            # gives back the number of walls
        return self.num_walls

    def get_spawn_zone(self):            # gives back the spawn_zone
        return self.spawn_zone

    def get_room_size(self):            # gives back the size of the room
        return self.room_size

    def get_destination(self):          # gives back the destination the agents want to get to
        return self.destination
