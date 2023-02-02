import numpy as np


class Utils:
    @staticmethod
    def string_to_float_list(d):
        return np.array(d.replace("[", "").replace("]", "").split(r", ")).astype(float)
