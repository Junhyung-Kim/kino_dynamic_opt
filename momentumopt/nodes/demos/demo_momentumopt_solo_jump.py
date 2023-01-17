#! /usr/bin/python

'''
@file demo_solo_jump.py
@package momentumopt
@author Brahayam Ponton (brahayam.ponton@tuebingen.mpg.de)
@license License BSD-3-Clause
@copyright Copyright (c) 2019, New York University and Max Planck Gesellschaft.
@date 2019-10-08
'''
import sys
sys.path.append('/usr/local/lib/python3/dist-packages')
sys.path.append('/home/jhk/kino_dynamic_opt/momentumopt/build/')
from os import path
from momentumopt.kino_dyn_planner import main

if __name__ == "__main__":

    cfg_solo_jump = path.join("..", "config", "cfg_solo12_jump.yaml")
    if(not path.exists(cfg_solo_jump)):
        raise RuntimeError("Please run this script from the demos folder")

    args = ["-i", cfg_solo_jump]

    main(args)