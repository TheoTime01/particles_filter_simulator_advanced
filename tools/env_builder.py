import numpy as np

from PIL import Image

def load_black_and_white_env(file_path_and_name: str):
    img = Image.open(file_path_and_name)
    numpydata = np.asarray(img)
    map_np=np.copy(numpydata)[:,:,0]
    map_np[map_np==0] = 1
    map_np[map_np!=1] = 0

    return map_np.T
