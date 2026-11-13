import argparse
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from greenhouse.navigation_api import request


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument("action",choices=("go","cancel","status"))
    parser.add_argument("--x",type=float)
    parser.add_argument("--y",type=float)
    parser.add_argument("--yaw",type=float,help="degrees counterclockwise from greenhouse +X")
    parser.add_argument("--speed",type=float,help="maximum drive speed in m/s (default: configured maximum)")
    parser.add_argument("--turn",type=float,help="maximum turn rate in rad/s (default: configured maximum)")
    args=parser.parse_args()
    try:
        print(json.dumps(request(args.action,{"x":args.x,"y":args.y,"yaw_deg":args.yaw,
                                                 "max_speed":args.speed,"max_turn":args.turn}),indent=2))
    except (ValueError,RuntimeError) as error:
        parser.exit(2,str(error)+"\n")


if __name__=="__main__":
    main()
