import argparse
import json
from pathlib import Path
from .evaluation import evaluate

def main():
    parser=argparse.ArgumentParser(description="Evaluate a policy in standalone simulation")
    parser.add_argument('--bundle',type=Path,required=True)
    parser.add_argument('--url')
    parser.add_argument('--device',choices=('cpu','mps','cuda'),default='cpu')
    parser.add_argument('--episodes',type=int,default=10)
    parser.add_argument('--paced',action='store_true')
    parser.add_argument('--output',type=Path,required=True)
    print(json.dumps(evaluate(**vars(parser.parse_args())),indent=2))

if __name__=='__main__':main()
