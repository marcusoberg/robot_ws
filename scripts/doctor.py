import importlib.util
import json
from robot_description.spec import so101
from robot_runtime_native._native import Simulation
p=so101();sim=Simulation(p.native_spec(),str(p.scene),p.initial)
sim.enable();session=sim.activate('doctor');snap=sim.step(p.initial,session,1)
assert snap.time==50_000_000 and len(snap.trace)==5 and snap.status.mode=='Running'
sim.disable()
print(json.dumps({'runtime':'v2','backend':p.backend,'action_spec':p.spec.identity,'ROS_installed':importlib.util.find_spec('rclpy') is not None,
                  'supported':['SO101 leader to MuJoCo','offline ACT','local/remote inference'],'deferred':['PAROL6','physical follower','Cartesian control']},indent=2))
