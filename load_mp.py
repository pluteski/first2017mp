#!/usr/bin/env python
# coding: utf-8
import argparse
import time
import logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
import motion_planner as mp
from motion_planner import StateP

#
#   Demonstrates how to load and manipulate a motion planning output file.
#
if __name__ == '__main__':
    logging.info("Running %s", __file__)
    start_t = time.time()
    parser = argparse.ArgumentParser(description='Demonstrates how to load states file.')
    parser.add_argument('--which','-w', default=mp.NORTH, help='Can be north or northeast.')
    parser.add_argument('--file','-f', required=True, help='Filepath.')
    parser.add_argument('--x','-x', action='store_true', help='Runs x')

    args = parser.parse_args()

    # Example config object with single row:
    example = {(0, 18, 304) : StateP((0, 19, 0), 20.1, 'rh', 15.0, (0, 19, 354), 2.2, None, 2)}
    logging.info("Example: %s", example)

    which = args.which
    config = mp.load_config(args.file)
    logging.info("Loaded %d rows", len(config))

    logging.info("")
    logging.info("Sample of first 5 rows: ")
    ii=0
    iter=config.iteritems()
    while ii < 5:
        k,v = next(iter)
        logging.info("%s: %s", k, v)
        ii+=1

    ii=0
    logging.info("")
    logging.info("Again, using pretty printer: ")
    iter=config.iteritems()
    while ii < 5:
        k,v = next(iter)
        logging.info("%s:", mp.str_state(k, v))
        ii+=1

    logging.info("")
    logging.info("First 5 rows having goal1=(46, -8, 300): ")
    ii=0
    for k,v in config.iteritems():
        if v.goal1() == (46, -8, 300):
            logging.info("%s", mp.str_state(k, v))
            ii+=1
            if ii>5:
                break
    logging.info("")
    logging.info("First 5 rows having type='|' : ")
    ii=0
    for k,v in config.iteritems():
        if v.type() == '|':
            logging.info("%s", mp.str_state(k, v))
            ii+=1
            if ii>5:
                break
    logging.info("")
    logging.info("Finished %s  (%.1fsec)", __file__, time.time()-start_t)

