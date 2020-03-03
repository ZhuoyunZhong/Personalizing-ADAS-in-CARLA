import os
from multiprocessing import Pool
from subprocess import Popen
import argparse


# Run two files simultaneously
def run_process(process):
    os.system('python {}'.format(process))


if __name__ == "__main__":
    # User interface
    argparser = argparse.ArgumentParser(description='CARLA Ego Vehicle Client')
    argparser.add_argument('-s', '--scene', metavar='int', default='1',
                           help='Scene Number')
    args = argparser.parse_args()

    # Run files according to scene number
    actor_file = 'load_actors_' + args.scene + '.py'
    processes = (actor_file, 'ego_vehicle.py')
    pool = Pool(processes=len(processes))
    pool.map(run_process, processes)
