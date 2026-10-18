import argparse
import json
from pathlib import Path

def smoke(args):
    from .application import Application
    import numpy as np
    root=Path(args.output)
    app=Application(root/'recordings',purpose='smoke')
    try:
        for episode in range(2):
            app.command('reset');app.activate('scripted-smoke');app.command('record')
            for i in range(12):
                q=np.array(app.env.profile.initial);q[0]=.05*np.sin((i+episode)*.15)
                app.step(q)
            app.recorder.finish(complete=True,reason='scripted_smoke')
        print(json.dumps({'recordings':str(root/'recordings'),'episodes':2,'learned_success_gate':'not evaluated'}),flush=True)
    finally:app.close()

def main():
    parser=argparse.ArgumentParser(description="Scripted simulation recording smoke test")
    parser.add_argument("--output",type=Path,required=True)
    smoke(parser.parse_args())

if __name__=="__main__":main()
