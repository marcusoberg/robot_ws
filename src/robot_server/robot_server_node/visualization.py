import threading
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor, ExternalShutdownException
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage
from robot_server.msg import Observation
import json

class Visualization(Node):
    def __init__(self, env, latest):
        super().__init__('robot_observations',start_parameter_services=False)
        self.env=env;self.latest=latest;self.spec=env.profile.spec
        qos=QoSProfile(reliability=ReliabilityPolicy.RELIABLE,history=HistoryPolicy.KEEP_LAST,depth=1)
        self.observations=self.create_publisher(Observation,'/robot/observation',qos)
        self.cameras={k:self.create_publisher(Image,f'/robot/cameras/{k}/rgb',qos) for k in self.spec.cameras}
        self.camera_infos={k:self.create_publisher(CameraInfo,f'/robot/cameras/{k}/rgb/camera_info',qos) for k in self.spec.cameras}
        self.transforms=self.create_publisher(TFMessage,'/tf',qos)
        self.timer=self.create_timer(.05,self.publish_observation)
        self.stop=threading.Event()
        self.publisher_executor=SingleThreadedExecutor();self.publisher_executor.add_node(self)
        self.thread=threading.Thread(target=self.run,name='ros-observations',daemon=True)
        self.thread.start()

    def run(self):
        try:
            while not self.stop.is_set() and rclpy.ok():
                self.publisher_executor.spin_once(timeout_sec=.05)
        except ExternalShutdownException:
            pass

    def destroy_node(self):
        self.stop.set();self.publisher_executor.wake();self.thread.join();self.publisher_executor.shutdown()
        super().destroy_node()

    @staticmethod
    def stamp(time_ns, header):
        header.stamp.sec,header.stamp.nanosec=divmod(time_ns,1_000_000_000)

    def observation(self, obs):
        message=Observation(episode=obs.episode,snapshot=obs.snapshot,acquisition_time_ns=obs.time_ns,
            host_acquired_ns=obs.host_acquired_ns,instance=obs.instance,action_spec_id=self.spec.identity,
            joints=list(self.spec.joints),measured_positions=obs.position.tolist(),measured_velocities=obs.velocity.tolist(),
            camera_metadata_json=json.dumps(obs.camera_metadata))
        for name,pixels in obs.images.items():
            image=Image(height=pixels.shape[0],width=pixels.shape[1],encoding='rgb8',is_bigendian=0,step=pixels.shape[1]*3,data=pixels.tobytes())
            self.stamp(obs.time_ns,image.header);image.header.frame_id=f'{name}_optical'
            message.camera_names.append(name);message.images.append(image)
        return message

    def publish_observation(self):
        obs=self.latest()
        if obs is None or obs.episode!=self.env.native.observe().episode:return
        message=self.observation(obs);self.observations.publish(message)
        transforms=[]
        for name,image in zip(message.camera_names,message.images):
            self.cameras[name].publish(image)
            meta=obs.camera_metadata[name]
            k=meta['intrinsics']
            info=CameraInfo(header=image.header,height=image.height,width=image.width,distortion_model='plumb_bob',
                            d=[0.]*5,k=k,r=[1.,0.,0.,0.,1.,0.,0.,0.,1.],p=[k[0],0.,k[2],0.,0.,k[4],k[5],0.,0.,0.,1.,0.])
            self.camera_infos[name].publish(info)
            tf=TransformStamped();tf.header.stamp=image.header.stamp;tf.header.frame_id='world';tf.child_frame_id=meta['frame']
            pose=meta['world_from_camera'];rotation=meta['rotation_xyzw']
            tf.transform.translation.x,tf.transform.translation.y,tf.transform.translation.z=(pose[i][3] for i in range(3))
            tf.transform.rotation.x,tf.transform.rotation.y,tf.transform.rotation.z,tf.transform.rotation.w=rotation
            transforms.append(tf)
        self.transforms.publish(TFMessage(transforms=transforms))
