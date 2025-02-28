from multiprocessing import shared_memory
import numpy as np
    
data = np.arange(100, dtype=np.int32)
shm = shared_memory.SharedMemory(create=True, size=data.nbytes, name="my_shared_data")
shared_array = np.ndarray(data.shape, dtype=data.dtype, buffer=shm.buf)
shared_array[:] = data[:]

