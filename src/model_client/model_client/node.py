import argparse
from concurrent.futures import ThreadPoolExecutor
import json
from pathlib import Path
import time

import numpy as np
import rclpy
from robot_description.spec import so101
from robot_server.msg import Observation as ObservationMessage
from robot_server_core.simulation import Observation, immutable
from robot_server_node.client import Source
from robot_inference.bundle import verify, load_local
from robot_inference.client import RemoteModel
from robot_inference.contracts import Features
from .policy import PolicyRunner


class PolicySource(Source):
    def __init__(self, bundle, url=None, device='cpu'):
        profile = so101()
        super().__init__('policy_source', profile.spec)
        manifest = verify(bundle)
        features = Features.from_json((bundle/'features.json').read_text())
        self.spec.require(features.spec)
        client = RemoteModel(url, features, expected_revision=manifest['revision']) if url else load_local(bundle, device)
        self.runner = PolicyRunner(self.spec, client)
        self.observation = None
        self.prime_pool = ThreadPoolExecutor(max_workers=1)
        self.priming = None
        self.requested = True
        self.started = False
        self.index = 0
        self.next_action = 0.
        self.create_subscription(ObservationMessage, '/robot/observation', self.receive_observation, 1)
        self.create_timer(.01, self.tick)

    def receive_observation(self, message):
        if message.action_spec_id != self.spec.identity or tuple(message.joints) != self.spec.joints:
            return
        if tuple(message.camera_names) != self.spec.cameras or len(message.images) != len(self.spec.cameras):
            return
        images = {}
        for name, image in zip(message.camera_names, message.images):
            stamp = image.header.stamp.sec*1_000_000_000 + image.header.stamp.nanosec
            if (image.encoding != 'rgb8' or (image.height,image.width,3) != self.spec.image_shape
                    or image.step != image.width*3 or stamp != message.acquisition_time_ns):
                return
            images[name] = immutable(np.frombuffer(image.data,dtype=np.uint8).reshape(self.spec.image_shape))
        self.observation = Observation(message.instance,message.episode,message.snapshot,message.acquisition_time_ns,
                                       message.host_acquired_ns,immutable(message.measured_positions),
                                       immutable(message.measured_velocities),images,json.loads(message.camera_metadata_json))

    def session_changed(self):
        if self.session:
            self.started = True
        elif getattr(self,'started',False):
            self.runner.reset()
            self.requested = False

    def tick(self):
        obs = self.observation
        if self.status is None or obs is None or (obs.instance,obs.episode,obs.time_ns) != (
                self.status.instance,self.status.episode,self.status.simulation_time_ns):
            return
        try:
            if self.requested and not self.started:
                if self.priming is None:
                    if self.status.mode != 'Ready' or self.status.session:
                        return
                    self.prime_basis = (obs.instance,obs.episode,obs.snapshot)
                    self.priming = self.prime_pool.submit(self.runner.prime,obs)
                elif self.priming.done():
                    self.priming.result()
                    if self.prime_basis != (obs.instance,obs.episode,obs.snapshot) or self.status.mode!='Ready' or self.status.session:
                        raise ValueError('Simulation changed while priming the policy')
                    self.control('activate')
                    self.requested = False
            if (self.started and self.available() and self.status.delivered_sequence >= self.sent_sequence
                    and time.monotonic() >= self.next_action):
                sample = self.runner.action(self.index,obs)
                if sample is None:
                    self.control('pause')
                    self.started = False
                    self.get_logger().info('Policy actions exhausted; source paused')
                elif self.send(sample.goal):
                    self.index += 1
                    self.next_action = time.monotonic() + self.spec.period_ns/1e9
        except Exception as exc:
            if self.session:
                self.control('pause')
            self.requested = self.started = False
            self.get_logger().error(f'Policy source ended: {exc}')

    def destroy_node(self):
        self.prime_pool.shutdown(wait=True,cancel_futures=True)
        self.runner.close()
        super().destroy_node()


def main():
    parser=argparse.ArgumentParser(description='Play policy actions through the ROS joint-goal topic')
    parser.add_argument('--bundle',type=Path,required=True)
    parser.add_argument('--url')
    parser.add_argument('--device',default='cpu',choices=('cpu','cuda','mps'))
    args,ros_args=parser.parse_known_args()
    rclpy.init(args=ros_args)
    node=None
    try:
        node=PolicySource(**vars(args))
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node:node.destroy_node()
        rclpy.try_shutdown()
