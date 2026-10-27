"""Passive telemetry subscription; no peer identity, lease or publisher."""
import json
from pathlib import Path
import time
import yaml
from elesim_protocol import DdsRuntimeSettings

path = Path('/home/idim/elesim/roles/robot/config/installed.yaml')
raw = yaml.safe_load(path.read_text())['dds']
vendor = Path(raw['vendor_config'])
raw['vendor_config'] = str(vendor if vendor.is_absolute() else path.parent/vendor)
settings = DdsRuntimeSettings.from_mapping(raw)
args = settings.apply_environment()
import rclpy
from elesim_interfaces.msg import PeerEnvelope
from rclpy.qos import QoSProfile, ReliabilityPolicy
rclpy.init(args=list(args))
node = rclpy.create_node('elesim_passive_collision_diagnostic', enable_rosout=False, start_parameter_services=False)
subscriptions, received = {}, []
def receive(msg):
    if msg.type == 'telemetry' and msg.source_id == 'robot-go2':
        received.append(json.loads(msg.payload_json))
deadline = time.monotonic()+12
try:
    while time.monotonic() < deadline and not received:
        for topic, types in node.get_topic_names_and_types():
            if topic.endswith('/control') and 'elesim_interfaces/msg/PeerEnvelope' in types and topic not in subscriptions:
                subscriptions[topic] = node.create_subscription(PeerEnvelope, topic, receive, QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE))
        rclpy.spin_once(node, timeout_sec=.1)
    if not received:
        raise SystemExit('No telemetry received')
    result = received[-1]
    Path(__file__).with_name('telemetry.json').write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps({key: result.get(key) for key in ('q', 'q_source', 'go2', 'claw_closed')}, indent=2))
finally:
    node.destroy_node()
    rclpy.shutdown()
