import os
import sys
from multiprocessing import Pool

# Run two files simultaneously
def run_process(process):
    os.system('python {}'.format(process))


if __name__ == "__main__":
    processes = ('load_actors.py', 'ego_vehicle.py')
    pool = Pool(processes=2)
    pool.map(run_process, processes)
