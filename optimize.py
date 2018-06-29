#!/usr/bin/env python3
import subprocess as sb
import numpy as np
from itertools import combinations_with_replacement


def run(p, i, d, sample_duration):
    output = sb.check_output(f"./build/pid {p} {i} {d} {sample_duration} > /dev/null", stderr=sb.STDOUT, shell=True)
    accumulated_error = float(output.decode().split("Accumulated error:")[1].strip())
    return accumulated_error

def run_repeat(p,i,d, repeat, sample_duration):
    average_error = np.array([run(p,i,d, sample_duration) for _ in range(repeat)]).mean()
    return average_error

def main():
    '''This script iteratively tries to find the optimum PID parameters.
    '''

    #pid = np.array([0.7, 0.003, 3.9]) # Initial parameter
    #pid = np.array([1.336956340797001, 0.005729812889130005, 16.291267860721053])
    #pid = np.array([1.3235867773890313, 0.005672514760238705, 19.712434111472476])
    #pid = np.array([0.9648947607166038,0.004135263260214017,23.8520452748817])
    #pid = np.array([0.8684052846449435, 0.004548789586235419, 26.23724980236987])
    pid = np.array([0.7815647561804492, 0.0040939106276118775, 31.747072260867547])

    increase_by = 0.1
    duration_samples = 4000
    repeat = 3 # each PID will be repeatedly run and an average taken

    # since we have 3 parameters, we create a cube of combinations to be able to select the best direction
    cube = list(combinations_with_replacement([1-increase_by,1,1+increase_by], 3))
    min_error = float("inf")
    [p_min, i_min, d_min] = pid # Initial pid
    while True:
        print(f"Trying PID: {p_min} {i_min} {d_min}")
        for index,c in enumerate(cube):
            pid_try = pid * c
            [p, i, d] = pid_try
            average_error = run_repeat(p, i, d, repeat, duration_samples)
            if average_error < min_error:
                min_error = average_error
                p_min, i_min, d_min = p, i, d
            print(f'[{index}] PID:[{p:.4f},{i:.4f},{d:.4f}] Error: {average_error:.4f}, Minimum error: {min_error:.2f} with {p_min:.4f}, {i_min:.4f}, {d_min:.4f}')
        pid = np.array([p_min, i_min, d_min])
if __name__ == '__main__':
    main()