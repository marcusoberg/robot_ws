import argparse
import json
from pathlib import Path

def main():
    parser=argparse.ArgumentParser(description='SO101 simulation v2. Physical followers, PAROL6 and Cartesian commands are not available.')
    sub=parser.add_subparsers(dest='command',required=True)
    p=sub.add_parser('replay');p.add_argument('--episode',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p=sub.add_parser('dataset');p.add_argument('--recordings',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--repo-id',required=True);p.add_argument('--smoke',action='store_true')
    p=sub.add_parser('train');p.add_argument('--dataset',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--device',choices=('cpu','mps','cuda'),required=True);p.add_argument('--steps',type=int,default=100000);p.add_argument('--validate-every',type=int,default=1000);p.add_argument('--smoke',action='store_true')
    p=sub.add_parser('export');p.add_argument('--checkpoint',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();kw=vars(args).copy();command=kw.pop('command')
    try:
        if command=='replay':
            from .replay import replay
            print(json.dumps(replay(**kw),indent=2))
        elif command=='dataset':
            from .dataset import convert
            print(json.dumps(convert(**kw),indent=2))
        elif command=='train':
            from .learning import train
            print(json.dumps(train(**kw),indent=2))
        elif command=='export':
            from robot_inference.bundle import export
            print(json.dumps(export(**kw),indent=2))
    except KeyboardInterrupt:
        parser.exit(130,"Stopped.\n")
    except (ValueError,RuntimeError,OSError) as exc:
        parser.exit(1,f'{exc}\n')
if __name__=='__main__':main()
