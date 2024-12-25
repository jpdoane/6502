#!/usr/bin/python3

import subprocess
import json

import os
import argparse

def tohex(v):
    if isinstance(v, int):
        return hex(v)
    elif isinstance(v, list):
        return "[" + ', '.join(tohex(x) for x in v) + "]"
    else:
        return "NaN"


def _runtest(test):
    init = test['initial']
    cycles = test['cycles']
    params=[]
    ramfile="test.bin"
    params.append("-d")
    params.append("-c")
    params.append(str(len(cycles)*2+3))
    if 'pc' in init:
        params.append("-j")
        params.append(str(init['pc']))
    if 'a' in init:
        params.append("-a")
        params.append(str(init['a']))
    if 's' in init:
        params.append("-s")
        params.append(str(init['s']))
    if 'x' in init:
        params.append("-x")
        params.append(str(init['x']))
    if 'y' in init:
        params.append("-y")
        params.append(str(init['y']))
    if 'p' in init:
        params.append("-p")
        params.append(str(init['p']))

    if 'ram' in init:
        ram = bytearray(2**16)
        for r in init['ram']:
            ram[r[0]] = r[1]


        # ram[0xFFFC] = 0 #;	// reset vector to 0x0000
        # ram[0xFFFD] = 0
        # ram[0] = 0xA9 #lda
        # ram[1] = init['a']
        # ram[2] = 0xA2 #ldx
        # ram[3] = init['s']
        # ram[4] = 0x9A #lxs
        # ram[5] = 0xA2 #ldx
        # ram[6] = init['x']
        # ram[7] = 0xA0 #ldy
        # ram[8] = init['y']
        # # ram[0x101 + init['s']] = init['p']  & 0xf7 #; // store p on stack
        # ram[0x101 + init['s']] = init['p'] #; // store p on stack
        # ram[9] = 0x28 #//plp: pop stack to p
        # ram[10] = 0x08 #//php: restore stack pointer
        # ram[11] = 0x4C #// jump to test program
        # ram[12] = init['pc'] & 0xFF
        # ram[13] = init['pc'] >> 8

        with open(ramfile, "wb") as fram:
            fram.write(ram)
        # print(f"Wrote ramfile {ramfile}")
        params.append(ramfile)
    return params

def runtest(testfile, testnum=0):
    simcmd = "/home/jpdoane/perfect6502/debug6502"
    with open(testfile) as json_data:
        tests = json.load(json_data)
        t = tests[testnum]
        print(f"Test {testnum} (of {len(tests)}): " + t['name'])
        print("Intial:", end=' ')
        init = t['initial']
        for k in init.keys():
            print( f"{k}:{tohex(init[k])}", end=' ')
        print( "" )
        params = _runtest(t)
        # print(simcmd + " " + " ".join(params))
        subprocess.call([simcmd] + params)
        print("Expected:", end=' ')
        final = t['final']
        for k in final.keys():
            print( f"{k}:{tohex(final[k])}", end=' ')
        print( "" )



parser = argparse.ArgumentParser()
parser.add_argument("testnum", type=int, help="number to run",default=0)
parser.add_argument("testop", help="operation to test (in hex)",default="69")
parser.add_argument("testpath", help="path to tests", default="./65x02/nes6502/v1")
args = parser.parse_args()

runtest(os.path.join(args.testpath, args.testop+".json"), args.testnum)


