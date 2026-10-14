"""Simulation mount command owner and measured, timestamped dynamic TF."""
import json
import math
from std_msgs.msg import Float64,String
from sensor_msgs.msg import JointState
from mavros_msgs.msg import State
from geometry_msgs.msg import TransformStamped
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import TransformBroadcaster,StaticTransformBroadcaster
from scipy.spatial.transform import Rotation
from .ros_nodes import ConfigNode,run,seconds,set_vector,set_quaternion
from .sensor_mounts import bounded_target,mount_rotation

class SensorMountNode(ConfigNode):
    def __init__(self):
        super().__init__('polinasi_sensor_mounts')
        self.declare_parameter('mode','ground_truth')
        if not self.c.get('sensor_gimbals_enabled') or (self.get_parameter('mode').value != 'ground_truth'
                and not (self.c.get('sensor_rolling_localisation') or self.c.get('sensor_fixed_localisation'))):
            raise ValueError('moving mounts currently require opt-in ground-truth simulation')
        self.tf = TransformBroadcaster(self)
        self.static = StaticTransformBroadcaster(self)
        optical = self.transform('zed_camera_link','zed_camera_optical',self.get_clock().now().to_msg(),
            [0.,0.,0.],Rotation.from_euler('xyz',[-math.pi/2,0.,-math.pi/2]).as_matrix())
        transforms = [optical]
        if self.c.get('lidar_fixed'):
            mount = self.c['lidar_mount']
            transforms.append(self.transform('base_link','mid360',optical.header.stamp,
                mount[:3],Rotation.from_euler('xyz',mount[3:]).as_matrix()))
        self.static.sendTransform(transforms)
        self.measured,self.stamps,self.targets,self.commands = {},{}, {},{}
        self.armed,self.state_stamp = None,-math.inf
        self.rejection = ''
        self.navigation = {}
        self.last_tick = self.now()
        self.outputs = {}
        self.rolling = self.c.get('simulation_continuous_roll',False)
        self.roll_rate_command = 0.
        self.roll_rate_measured = None
        if self.rolling:
            self.targets['lidar_roll'] = 0.
            self.roll_output = self.create_publisher(Float64,'/lidar/roll_rate_cmd',1)
        for joint,topic in [('lidar_pitch','/lidar/pitch_cmd'),('camera_pitch','/camera/pitch_cmd'),('camera_yaw','/camera/yaw_cmd')]:
            if joint=='lidar_pitch' and (self.rolling or self.c.get('lidar_fixed')):
                continue
            if joint=='camera_yaw' and not self.c.get('camera_yaw_enabled',True):
                continue
            self.targets[joint] = self.commands[joint] = 0.
            self.outputs[joint] = self.create_publisher(Float64,topic,1)
            target_topic = '/mounts/'+joint.replace('_','/')+'_target'
            self.create_subscription(Float64,target_topic,lambda msg,j=joint:self.target(j,msg.data),1)
        self.create_subscription(JointState,'/camera/joint_state',self.feedback,qos_profile_sensor_data)
        self.create_subscription(JointState,'/lidar/joint_state',self.feedback,qos_profile_sensor_data)
        self.create_subscription(State,'/mavros/state',self.state,qos_profile_sensor_data)
        self.create_subscription(String,'/navigation/status',lambda m:setattr(self,'navigation',json.loads(m.data)),1)
        self.status = self.create_publisher(String,'/mounts/status',1)
        self.create_timer(.05,self.tick)

    def state(self,msg):
        self.armed,self.state_stamp = msg.armed,self.now()

    def target(self,joint,value):
        try:
            target = bounded_target(joint,value)
        except ValueError as exc:
            self.rejection = str(exc)
            return
        if joint == 'lidar_pitch' and abs(target)>1e-6 and (self.armed is not False or self.now()-self.state_stamp>1.):
            nav = getattr(self,'navigation',{})
            permitted = self.c.get('automatic_tilt_ascent',False) and self.armed is True and self.now()-self.state_stamp<1. and 0<=self.now()-nav.get('simulation_time',-math.inf)<.3 and nav.get('lidar_tilt_fsm_authorized') and nav.get('flight')=='MISSION' and not nav.get('failure')
            unchanged = abs(target-self.targets[joint])<1e-6
            if not permitted or (not unchanged and not nav.get('lidar_tilt_change_authorized')):
                self.rejection = 'lidar_tilt_requires_disarmed_or_authorized_stationary_FSM'
                return
        self.targets[joint] = target
        self.rejection = ''

    def transform(self,parent,child,stamp,xyz,matrix):
        t = TransformStamped()
        t.header.stamp,t.header.frame_id,t.child_frame_id = stamp,parent,child
        set_vector(t.transform.translation,xyz)
        set_quaternion(t.transform.rotation,matrix)
        return t

    def feedback(self,msg):
        stamp = seconds(msg.header.stamp)
        if not 0 <= self.now()-stamp < .3:
            return
        names = {name.split('::')[-1]:i for i,name in enumerate(msg.name)}
        updated = []
        for joint in self.targets:
            index = names.get(joint)
            if index is None or index>=len(msg.position) or not math.isfinite(msg.position[index]):
                continue
            if stamp-self.stamps.get(joint,-math.inf)<.019:
                continue
            if joint=='lidar_roll':
                previous_stamp=self.stamps.get(joint)
                previous_angle=self.measured.get(joint)
                if previous_stamp is not None and stamp>previous_stamp:
                    self.roll_rate_measured=(msg.position[index]-previous_angle)/(stamp-previous_stamp)
            self.measured[joint],self.stamps[joint] = msg.position[index],stamp
            updated.append(joint)
        transforms = []
        if 'lidar_pitch' in updated:
            mount = self.c['lidar_mount']
            matrix = Rotation.from_euler('xyz',mount[3:]).as_matrix()@mount_rotation(pitch=self.measured['lidar_pitch'])
            transforms.append(self.transform('base_link','mid360',msg.header.stamp,mount[:3],matrix))
        if 'lidar_roll' in updated:
            mount = self.c['lidar_mount']
            matrix = Rotation.from_euler('xyz',mount[3:]).as_matrix()@Rotation.from_euler('x',self.measured['lidar_roll']).as_matrix()
            transforms.append(self.transform('base_link','mid360',msg.header.stamp,mount[:3],matrix))
        if 'camera_pitch' in updated and ('camera_yaw' in updated or not self.c.get('camera_yaw_enabled',True)):
            transforms.extend([
                self.transform('base_link','camera_yaw_link',msg.header.stamp,self.c['camera_mount'],mount_rotation(yaw=self.c.get('camera_mount_yaw',0.)+self.measured.get('camera_yaw',0.))),
                self.transform('camera_yaw_link','zed_camera_link',msg.header.stamp,[0.,0.,0.],mount_rotation(pitch=self.measured['camera_pitch']))])
        if transforms:
            self.tf.sendTransform(transforms)

    def tick(self):
        now = self.now()
        dt = max(0.,min(.1,now-self.last_tick))
        self.last_tick = now
        nav = self.navigation
        permission = self.c.get('automatic_tilt_ascent',False) and self.armed is True and now-self.state_stamp<1. and 0<=now-nav.get('simulation_time',-math.inf)<.3 and nav.get('lidar_tilt_fsm_authorized') and nav.get('flight')=='MISSION' and not nav.get('failure')
        if not getattr(self,'rolling',False) and ((self.armed is not False and not permission) or now-self.state_stamp>1.):
            if 'lidar_pitch' in self.targets:
                self.targets['lidar_pitch'] = 0.
        if getattr(self,'rolling',False):
            fresh_nav = 0 <= now-nav.get('simulation_time',-math.inf) < .3
            allowed = now-self.state_stamp<1. and fresh_nav and not nav.get('failure')
            self.roll_rate_command = self.c['simulation_roll_rate'] if allowed else 0.
            self.roll_output.publish(Float64(data=float(self.roll_rate_command)))
        for joint,pub in self.outputs.items():
            rate = .35 if joint=='lidar_pitch' else .5
            delta = max(-rate*dt,min(rate*dt,self.targets[joint]-self.commands[joint]))
            self.commands[joint] += delta
            pub.publish(Float64(data=self.commands[joint]))
        fresh = all(0<=now-self.stamps.get(j,-math.inf)<.3 for j in self.targets)
        ready = fresh and (self.c.get('lidar_fixed') or getattr(self,'rolling',False) or (abs(self.measured.get('lidar_pitch',math.inf))<.02 and abs(self.targets['lidar_pitch'])<1e-6))
        self.status.publish(String(data=json.dumps(dict(stamp=now,measured=self.measured,
            targets=self.targets,commands=self.commands,feedback_fresh=fresh,navigation_ready=ready,
            rejection=self.rejection,airborne_lidar_tilt_supported=bool(self.c.get('automatic_tilt_ascent')),
            fsm_authorization_fresh=bool(permission),
            continuous_roll_simulation_only=getattr(self,'rolling',False),
            roll_rate_command_rad_s=getattr(self,'roll_rate_command',0.),
            roll_rate_measured_rad_s=getattr(self,'roll_rate_measured',None)),allow_nan=False)))

def main(args=None):
    run(SensorMountNode,args)
