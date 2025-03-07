import numpy as np


class Utils:
    def generate_random_color():
        return list(np.random.choice(range(0, 200), size=3))

    def get_random_rviz_color():
        r, g, b = Utils.generate_random_color()

        return f"{r}; {g}; {b}"