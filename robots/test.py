import argparse

argparser = argparse.ArgumentParser()
argparser.add_argument("func", type=str)
argparser.add_argument("code", type=str)

FUNCTIONS = """
MOVE(direction:str)
direction must be one of ["FORWARD", "BACKWARD", "LEFT", "RIGHT"]
"""
SUGGESTIONS = """
MOVE,FORWARD
MOVE,LEFT
"""

def move(direction: str) -> str:
    return f"moving {direction}"

if __name__ == "__main__":
    args = argparser.parse_args()
    if args.func.upper() == "MOVE":
        print(move(args.code))
    else:
        print(f"Unknown func {args.func} code {args.code}")
