import argparse
import os
from pathlib import Path
import subprocess
import sys
ROOT=Path(__file__).resolve().parents[1]
def main():
    parser=argparse.ArgumentParser(description='Separate offline ACT learning and inference environment')
    parser.add_argument('operation',choices=('setup','dataset','train','export','serve','evaluate'))
    args,extra=parser.parse_known_args()
    env=os.environ.copy();env['UV_CACHE_DIR']=str(ROOT/'.workspace/uv-cache')
    if args.operation=='setup':
        subprocess.run(['uv','sync','--project',str(ROOT/'environments/models'),*extra],env=env,cwd=ROOT,check=True)
        return
    python=ROOT/'environments/models/.venv/bin/python'
    if not python.exists():parser.error('Run setup-models first')
    env['PYTHONPATH']=os.pathsep.join(str(ROOT/p) for p in ('src/robot_description','src/robot_data','src/robot_server','src/model_client','external/robot_runtime/python','install/standalone/python'))
    for key in ('AMENT_PREFIX_PATH','COLCON_PREFIX_PATH','ROS_DISTRO','ROS_VERSION','ROS_PYTHON_VERSION','DYLD_LIBRARY_PATH'):
        env.pop(key,None)
    command=['-m','robot_inference.server'] if args.operation=='serve' else ['-m','model_client.cli'] if args.operation=='evaluate' else ['-m','robot_data.cli',args.operation]
    subprocess.run([str(python),*command,*extra],env=env,cwd=ROOT,check=True)
if __name__=='__main__':
    try:main()
    except KeyboardInterrupt:raise SystemExit(130)
