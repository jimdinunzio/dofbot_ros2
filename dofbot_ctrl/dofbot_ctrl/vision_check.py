#!/usr/bin/env python3
# coding: utf-8
"""
Ask the arm camera one question and print the answer.

    ros2 run dofbot_ctrl vision_check -- --held
    ros2 run dofbot_ctrl vision_check -- --approach --object test_block
    ros2 run dofbot_ctrl vision_check -- --held --label 'a beer can'

This is the process `arm_server.is_holding()` runs. It takes no ROS parameters
and builds no node -- there is nothing to plan, only a camera to open and a
detector to ask -- but it lives behind `ros2 run` like every other command the
arm service issues, so that one sourced workspace serves all of them.

EXIT STATUS IS NOT THE VERDICT. 0 means the question was asked and answered,
whatever the answer was; 'the can is not there' is a successful query with a
negative result, not a failed command. 2 is a usage error and 1 is a failure to
run at all. The verdict is in the machine-readable line:

    VERDICT: {"verdict": "present", "check": "held", ...}

which is the one line of this program's output meant to be parsed. Everything
else is for a human, and follows the same rule as the rest of the stack: hand
it over, do not scrape it.
"""

import argparse
import json
import sys

from dofbot_ctrl import graspable, vision

# The prefix arm_server looks for. Changing it changes the wire format between
# two files that are versioned together; changing it in only one of them makes
# is_holding() answer 'unknown' forever, and quietly.
VERDICT_PREFIX = 'VERDICT: '


def main(argv=None):
    parser = argparse.ArgumentParser(
        prog='vision_check', description=__doc__.split('\n\n')[0])
    which = parser.add_mutually_exclusive_group()
    which.add_argument('--held', action='store_true',
                       help='is the object still in the jaws? (the default)')
    which.add_argument('--approach', action='store_true',
                       help='is the object where the pick was told it is? '
                            'Meaningful only from a pre-grasp standoff')
    parser.add_argument('--object', default='soda_can',
                        help='catalogue entry to look for (default soda_can)')
    parser.add_argument('--label', default='',
                        help='prompt the detector with this instead of the '
                             'label derived from the catalogue name')
    parser.add_argument('--url', default=vision.OWL_URL,
                        help='nanoOWL XML-RPC server (default %(default)s)')
    parser.add_argument('--quiet', action='store_true',
                        help='print only the VERDICT line')
    cli = parser.parse_args(sys.argv[1:] if argv is None else argv)

    check = 'approach' if cli.approach else 'held'

    if cli.label:
        label = cli.label
    else:
        try:
            label = vision.label_for(graspable.get(cli.object))
        except graspable.ObjectError as exc:
            print(exc, file=sys.stderr)
            return 2

    sighting = vision.look(check, label, url=cli.url)

    if not cli.quiet:
        print(vision.describe(sighting))
    print(VERDICT_PREFIX + json.dumps(vision.as_dict(sighting)))
    return 0


if __name__ == '__main__':
    sys.exit(main())
