import argparse
import os
from pathlib import Path
import subprocess
import sys

ROOT=Path(__file__).resolve().parents[1]
def run(*args,env=None):
    subprocess.run([str(x) for x in args],cwd=ROOT,env=env,check=True)

def main():
    p=argparse.ArgumentParser(description='Build the standalone runtime. ROS is opt-in; learning dependencies are separate.')
    p.add_argument('--jobs',type=int,default=2);p.add_argument('--build-only',action='store_true');p.add_argument('--ros',action='store_true')
    a=p.parse_args()
    if a.jobs<1:p.error('--jobs must be positive')
    prefix=ROOT/'install/standalone'
    build=ROOT/'build'/f'standalone-{Path(sys.prefix).name}'
    if not a.build_only:
        run(sys.executable,'-m','pip','install','--no-build-isolation','-e','src/robot_description','-e','src/robot_data','-e','src/robot_server','-e','src/robot_teleop','-e','src/model_client','-e','external/robot_runtime/python')
    run('cmake','-S','external/robot_runtime','-B',build,'-DCMAKE_BUILD_TYPE=Release',
        '-DROBOT_RUNTIME_WITH_PYTHON=ON',f'-DPython_EXECUTABLE={sys.executable}',f'-DCMAKE_INSTALL_PREFIX={prefix}')
    run('cmake','--build',build,'-j',a.jobs)
    run('ctest','--test-dir',build,'--output-on-failure')
    run('cmake','--install',build)
    import sysconfig
    (Path(sysconfig.get_path('purelib'))/'robot_runtime_native_v2.pth').write_text(str(prefix/'python')+'\n')
    if a.ros:
        run('colcon','build','--base-paths','src/robot_description','src/robot_data','src/robot_server','src/robot_teleop','src/model_client','src/robot_bringup','--build-base','build/ros-v2','--install-base','install/ros-v2',
            '--merge-install','--symlink-install','--executor','sequential','--cmake-args',f'-DPython3_EXECUTABLE={sys.executable}')
    print('Standalone runtime installed. SO101 and PAROL6 simulation available; physical followers are deferred.')
if __name__=='__main__':main()
