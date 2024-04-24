#!/usr/bin/env python3

import numpy as np
import roslib
import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Bool, String
from rovi_industrial import rtde_sock as comm
from rovi_utils import tflib
from scipy.spatial.transform import Rotation as R
import time
import sys
import json

Config={
  'robot_ip':'127.0.0.1',
  'joint_ids':['shoulder_pan_joint','shoulder_lift_joint','elbow_joint','wrist_1_joint','wrist_2_joint','wrist_3_joint'],
  'robot_recipe':'default.xml',
  'robot_port':30004,
  'tcp0_frame_id':'tool0_controller',
  'publish_tick': 5,
  'copy':[
    {'param':'/dashboard/ind/rsocket/enable','input':'input_bit_register_64'},
    {'param':'/dashboard/ind/rovi/stat','input':'input_bit_register_65'},
    {'param':'/dashboard/ind/diskfree/stat','input':'input_bit_register_66'},
    {'state':'payload_inertia[0]','input':'input_int_register_24','gain':10},
    { 'state': 'input_int_register_20', 'publish': '/report', 'key': 'CaptRowNo' },
    { 'state': 'input_int_register_21', 'publish': '/report', 'key': 'CaptColNo' },
    { 'state': 'input_int_register_22', 'publish': '/report', 'key': 'ReCaptNo' },
  ],
}

__PUBLISH_TICK__ = Config["publish_tick"]

rospy.init_node('rclient_ur',anonymous=True)
joints=JointState()
joints.name=Config['joint_ids']
try:
  Config.update(rospy.get_param('/config/rsocket'))
except Exception as e:
  print("get_param exception:",e.args)
pub_js=rospy.Publisher('/joint_states',JointState,queue_size=1)
pub_tf=rospy.Publisher('/update/config_tf',TransformStamped,queue_size=1)
pub_conn=rospy.Publisher('/rsocket/enable',Bool,queue_size=1)

# Dynamic Publishers
class Publishers:
  def __init__(self, tick: int):
    self.publishers = []
    self.tick = tick
  def set(self, endpoint, key, value):
    pub = next(filter(lambda x: x.endpoint == endpoint, self.publishers), None)
    if pub == None:
      pub = self.create_endpoint(endpoint, self.tick)
    pub.add_payload(key, value)
  def create_endpoint(self, endpoint, tick):
    pub = CustomPublisher(endpoint, tick)
    self.publishers.append(pub)
    return pub
  def next_tick(self):
    for pub in self.publishers:
      pub.next_tick()
    
class CustomPublisher:
  def __init__(self, endpoint: str, tick: int):
    self.endpoint = endpoint
    self.tick = tick
    self.tick_count = 0
    self.publisher = rospy.Publisher(endpoint, String, queue_size=1)
    self.cached_payloads = []
    self.payload = {}
  def next_tick(self):
    self.tick_count = self.tick_count + 1
    if self.tick_count >= self.tick:
      self.publisher.publish(json.dumps(self.payload))
      self.tick_count = 0
      self.clear_cache()
    self.clear_payload()
  def add_payload(self, key, value):
    self.payload[key] = value
  def clear_payload(self):
    self.cached_payloads.append(self.payload)
    self.payload = {}
  def clear_cache(self):
    self.cached_payloads = []
    
pubs = Publishers(__PUBLISH_TICK__)

print("rclient_ur::",Config['robot_ip'])

mTrue=Bool();mTrue.data=True
mFalse=Bool();mFalse.data=False

def update(q):
  if len(q)>=len(joints.name):
    joints.header.stamp=rospy.Time.now()
    joints.position=q
    pub_js.publish(joints)
    pub_conn.publish(mTrue)

while True:
  if rospy.is_shutdown(): sys.exit()
  rospy.sleep(3)

  if not comm.connect(Config['robot_ip'],Config['robot_port'],Config['robot_recipe'],uplink=True):
    print("rclient_ur::comm connect falied",Config['robot_ip'])
    continue
  if not comm.start():
    print("rclient_ur::comm start falied")
    continue

  print("rclient_ur::comm connected",Config['robot_ip'])

###Code generator##############
  pycode=''
  for obj in Config['copy']:
    if 'publish' in obj:
      if len(pycode)>0: pycode=pycode+'\n'
      pycode=pycode+'pubs.set("' + obj["publish"] +'","' + obj["key"] + '", comm.state.' + obj['state'] + ')'
      continue
    
    lvar='comm.inregs.'+obj['input']
    exec(lvar+'=0')
    if 'param' in obj:
      if len(pycode)>0: pycode=pycode+'\n'
      pycode=pycode+lvar+'=rospy.get_param("'+obj['param']+'")'
    elif 'state' in obj:
      if len(pycode)>0: pycode=pycode+'\n'
      pycode=pycode+lvar+'=int(comm.state.'+obj['state']+'*'+str(obj['gain'])+')'
      
  if len(pycode)>0: pycode=pycode+'\n'      
  pycode=pycode+'pubs.next_tick()'
  
  print('Code generator\n',pycode)
###Start Event Loop##############
  try:
    conf_tf=rospy.get_param('/config_tf')
  except Exception as e:
    conf_tf={}
  tcp0_id=Config['tcp0_frame_id']
  loop=True
  while True:
    rospy.sleep(0.05)
    if rospy.is_shutdown():
      loop=False
      break
    try:
      comm.receive()
      update(comm.state.actual_q)
      print("inreg",comm.state.input_int_register_20)
      if len(pycode)>0:
        try:
          exec(pycode)
        except Exception as e:
          print("Exec error",e,file=sys.stderr)
      comm.send()
      if tcp0_id in conf_tf:
        tf=TransformStamped()
        tf.header.stamp=rospy.Time.now()
        tf.header.frame_id=conf_tf[tcp0_id]['parent_frame_id']
        tf.child_frame_id=tcp0_id
        bTp=np.eye(4)   #base To tcp
        bTp[:3,:3]=R.from_rotvec(comm.state.actual_TCP_pose[3:]).as_matrix()
        bTp[:3,3]=np.array(comm.state.actual_TCP_pose[:3])*1000
        fTp=np.eye(4)   #frange To tcp
        fTp[:3,:3]=R.from_rotvec(comm.state.tcp_offset[3:]).as_matrix()
        fTp[:3,3]=np.array(comm.state.tcp_offset[:3])*1000
        tf.transform=tflib.fromRT(bTp.dot(tflib.invRT(fTp)))
        pub_tf.publish(tf);
    except Exception as e:
      print('rclient_ur::rtde failed',e)
      break

  try:
    comm.pause()
  except Exception as e:
    print('rclient_ur::rtde stop1',e)

  try:
    comm.disconnect()
  except Exception as e:
    print('rclient_ur::rtde stop2',e)

  if not loop: sys.exit()

