#!/usr/bin/env python3
"""Real Gazebo Harmonic + PX4 SITL ground recovery. No real vehicle interfaces.

Uses the stock x500 mass, inertia, collisions and signed-speed motor plugins.
All assertions use Gazebo ground truth as well as the actual PX4 mode/outputs.
"""
import argparse
import csv
import hashlib
import json
import math
import os
import re
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time
import xml.etree.ElementTree as ET

sys.path.append('/usr/lib/python3/dist-packages')
from gz.transport13 import Node
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.actuators_pb2 import Actuators
from gz.msgs10.contacts_pb2 import Contacts
from pymavlink import mavutil

REPO = Path(__file__).resolve().parents[2]


def guarded_model(directory):
    """Explicit square upper guard assumption; stock rotors/collisions remain intact."""
    directory.mkdir(parents=True, exist_ok=True)
    base = ET.parse(REPO/'Tools/simulation/gz/models/x500_base/model.sdf')
    model = base.getroot().find('model')
    model.set('name', 'x500_guarded_base')
    guard = ET.SubElement(model, 'link', name='guard')
    ET.SubElement(guard, 'pose').text = '0 0 .15 0 0 0'
    inertia = ET.SubElement(guard, 'inertial')
    ET.SubElement(inertia, 'mass').text = '.12'
    tensor = ET.SubElement(inertia, 'inertia')
    for key, value in [('ixx', '.0104'), ('iyy', '.0104'), ('izz', '.0208'), ('ixy', '0'), ('ixz', '0'), ('iyz', '0')]:
        ET.SubElement(tensor, key).text = value
    shapes = []
    for y in [-.36, .36]:
        shapes.append((f'0 {y} 0 0 {math.pi/2} 0', 'cylinder'))
    for x in [-.36, .36]:
        shapes.append((f'{x} 0 0 {math.pi/2} 0 0', 'cylinder'))
    for x in [-.36, .36]:
        for y in [-.36, .36]:
            shapes.append((f'{x} {y} 0 0 0 0', 'sphere'))
    for index, (pose, shape) in enumerate(shapes):
        for kind in ['collision', 'visual']:
            item = ET.SubElement(guard, kind, name=f'guard_{index}')
            ET.SubElement(item, 'pose').text = pose
            geometry = ET.SubElement(ET.SubElement(item, 'geometry'), shape)
            ET.SubElement(geometry, 'radius').text = '.025'
            if shape == 'cylinder':
                ET.SubElement(geometry, 'length').text = '.72'
            if kind == 'collision':
                ode = ET.SubElement(ET.SubElement(ET.SubElement(item, 'surface'), 'friction'), 'ode')
                ET.SubElement(ode, 'mu').text = '.8'
                ET.SubElement(ode, 'mu2').text = '.8'
    joint = ET.SubElement(model, 'joint', name='guard_fixed', type='fixed')
    ET.SubElement(joint, 'parent').text = 'base_link'
    ET.SubElement(joint, 'child').text = 'guard'
    def contact_sensor(link, name, collisions):
        sensor = ET.SubElement(link, 'sensor', name=name, type='contact')
        ET.SubElement(sensor, 'always_on').text = 'true'
        ET.SubElement(sensor, 'update_rate').text = '500'
        ET.SubElement(sensor, 'topic').text = f'/turtle/{name}'
        contact = ET.SubElement(sensor, 'contact')
        # Gazebo 8 Contact reads the legacy nested topic; retain sensor/topic too.
        ET.SubElement(contact, 'topic').text = f'/turtle/{name}'
        for collision in collisions:
            ET.SubElement(contact, 'collision').text = collision
    contact_sensor(guard, 'guard_contacts', [f'guard_{i}' for i in range(8)])
    for motor in range(4):
        contact_sensor(model.find(f"link[@name='rotor_{motor}']"), f'rotor{motor}_contacts', [f'rotor_{motor}_collision'])
    base_dir = directory/'x500_guarded_base'
    base_dir.mkdir()
    base.write(base_dir/'model.sdf')
    top = ET.parse(REPO/'Tools/simulation/gz/models/x500/model.sdf')
    top.getroot().find('model').set('name', 'x500_guarded')
    top.getroot().find('.//include/uri').text = 'model://x500_guarded_base'
    top_dir = directory/'x500_guarded'
    top_dir.mkdir()
    top.write(top_dir/'model.sdf')
    for name in ['x500_guarded_base', 'x500_guarded']:
        (directory/name/'model.config').write_text(f'<model><name>{name}</name><version>1</version><sdf version="1.9">model.sdf</sdf></model>')
    return {'type': 'square upper guard', 'bar_center_span_m': .72, 'tube_radius_m': .025,
            'center_height_above_base_m': .15, 'mass_kg': .12,
            'inertia_kg_m2': [.0104, .0104, .0208],
            'total_model_mass_kg': sum(float(e.text) for e in model.findall('.//inertial/mass')), 'assumptions': 'not measured from the user vehicle; stock base/motors/rotor collisions retained'}


def world_text(slope_roll, slope_pitch, yaw, model, upright=False):
    plugins = ''.join(f'<plugin filename="gz-sim-{file}-system" name="gz::sim::systems::{name}"/>' for file, name in (
        ('physics', 'Physics'), ('contact', 'Contact'), ('user-commands', 'UserCommands'), ('scene-broadcaster', 'SceneBroadcaster'),
        ('imu', 'Imu'), ('air-pressure', 'AirPressure'), ('navsat', 'NavSat'), ('magnetometer', 'Magnetometer')))
    return f'''<sdf version="1.9"><world name="turtle">
    {plugins}<physics name="physics" type="ignored"><max_step_size>0.002</max_step_size><real_time_factor>1</real_time_factor></physics>
    <gravity>0 0 -9.81</gravity><magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>
    <spherical_coordinates><surface_model>EARTH_WGS84</surface_model><world_frame_orientation>ENU</world_frame_orientation><latitude_deg>47.397971</latitude_deg><longitude_deg>8.546164</longitude_deg><elevation>488</elevation></spherical_coordinates>
    <model name="ground"><static>true</static><pose>0 0 0 {slope_roll} {slope_pitch} 0</pose><link name="ground_link">
    <collision name="ground_collision"><geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry><surface><friction><ode><mu>0.8</mu><mu2>0.8</mu2></ode></friction></surface></collision>
    <visual name="ground_visual"><geometry><plane><normal>0 0 1</normal><size>20 20</size></plane></geometry></visual></link></model>
    <include><uri>model://{model}</uri><name>x500_0</name><pose>0 0 0.6 {slope_roll if upright else math.radians(170)} {slope_pitch if upright else 0} {math.radians(90-yaw)}</pose></include>
    </world></sdf>'''


class Experiment:
    def __init__(self, args):
        self.args = args
        self.out = args.output.resolve()
        if self.out.exists() and any(self.out.iterdir()):
            raise ValueError('Use a new or empty output directory to keep each run isolated')
        self.out.mkdir(parents=True, exist_ok=True)
        self.rows, self.motor_rows = [], []
        self.guard_contacts = self.rotor_contacts = 0
        self.contact_messages = 0
        self.latest_pose, self.latest_velocity = None, [0.0]*4
        self.heartbeat = None
        self.mode_channel = 1000
        self.stop_radio = threading.Event()
        self.start = time.monotonic()
        self.gz = self.px4 = None
        self.env = os.environ.copy()
        self.env.update(GZ_PARTITION=f'rm27-turtle-{os.getpid()}', GZ_IP='127.0.0.1',
                        GZ_SIM_RESOURCE_PATH=str(REPO/'Tools/simulation/gz/models'))
        os.environ.update({k: self.env[k] for k in ('GZ_PARTITION', 'GZ_IP')})
        self.node = Node()
        self.node.subscribe(Pose_V, '/world/turtle/pose/info', self.pose_callback)
        self.node.subscribe(Actuators, '/x500_0/command/motor_speed', self.motor_callback)
        self.node.subscribe(Contacts, '/turtle/guard_contacts', lambda msg: self.contact_callback(msg, False))
        for motor in range(4):
            self.node.subscribe(Contacts, f'/turtle/rotor{motor}_contacts', lambda msg: self.contact_callback(msg, True))
        self.connection = mavutil.mavlink_connection('udpin:127.0.0.1:14540', source_system=255, source_component=190)

    def pose_callback(self, msg):
        for pose in msg.pose:
            if pose.name != 'x500_0':
                continue
            q = pose.orientation
            # R33 is invariant to the ENU/FLU <-> NED/FRD double-Z conversion.
            gz = 1-2*(q.x*q.x+q.y*q.y)
            tilt = math.degrees(math.acos(max(-1, min(1, gz))))
            self.latest_pose = (tilt, pose.position.x, pose.position.y, pose.position.z, q.w, q.x, q.y, q.z)
            self.rows.append([time.monotonic()-self.start, *self.latest_pose, *self.latest_velocity])

    def motor_callback(self, msg):
        self.latest_velocity = list(msg.velocity)[:4]
        self.motor_rows.append([time.monotonic()-self.start, *self.latest_velocity])

    def contact_callback(self, msg, rotor):
        self.contact_messages += 1
        for contact in msg.contact:
            if 'ground' in contact.collision1.name or 'ground' in contact.collision2.name:
                if rotor:
                    self.rotor_contacts += 1
                else:
                    self.guard_contacts += 1

    def run_for(self, seconds):
        deadline = time.monotonic()+seconds
        while time.monotonic() < deadline:
            msg = self.connection.recv_match(blocking=True, timeout=.05)
            if msg and msg.get_type() == 'HEARTBEAT' and msg.autopilot == mavutil.mavlink.MAV_AUTOPILOT_PX4:
                self.heartbeat = msg
                if msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
                    raise AssertionError('Recovery unexpectedly armed normal flight')
            if self.gz.poll() is not None or self.px4 and self.px4.poll() is not None:
                raise RuntimeError('A simulation process exited; inspect logs')

    def radio_loop(self):
        count = 0
        while not self.stop_radio.is_set():
            if count % 25 == 0:
                self.connection.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, mavutil.mavlink.MAV_STATE_ACTIVE)
            count += 1
            self.connection.mav.rc_channels_override_send(0, 0, 1500, 1500, 1000, 1500, self.mode_channel, 1000, 1000, 1000)
            self.stop_radio.wait(.04)

    def parameter(self, name, value):
        self.console(f'param set {name} {value}')
        self.run_for(.08)

    def console(self, command):
        module, *arguments = command.split()
        result = subprocess.run([str(REPO/'build/px4_sitl_default/bin'/f'px4-{module}'), *arguments], env=self.env,
                                capture_output=True, text=True, timeout=3)
        with (self.out/'console.log').open('a') as log:
            log.write(f'>>> {command}\n{result.stdout}\n{result.stderr}\n')
        return result.stdout+result.stderr

    def run(self):
        world = self.out/'world.sdf'
        guard = guarded_model(self.out/'models') if self.args.model == 'guarded' else None
        model = 'x500_guarded' if guard else 'x500'
        if guard:
            self.env['GZ_SIM_RESOURCE_PATH'] = str(self.out/'models')+os.pathsep+self.env['GZ_SIM_RESOURCE_PATH']
        world.write_text(world_text(math.radians(self.args.slope_roll), math.radians(self.args.slope_pitch), self.args.yaw, model, self.args.upright))
        logs = [open(self.out/'gazebo.log', 'w'), open(self.out/'px4.log', 'w')]
        result = {'case': self.args.case, 'power': self.args.power, 'pulse': self.args.pulse, 'timeout': self.args.timeout, 'rate_target': self.args.rate, 'level_pulse': self.args.level_pulse,
                  'slope_degrees': [self.args.slope_roll, self.args.slope_pitch], 'yaw_degrees': self.args.yaw,
                  'model': self.args.model, 'guard_assumptions': guard, 'first_coast_degrees': 60, 'completion_degrees': 1 if self.args.upright else 30, 'upright': self.args.upright, 'expect_timeout': self.args.expect_timeout, 'stock_base': 'x500, original base/motor/rotor physics; guarded adds mass/inertia/collisions', 'fault': self.args.fault, 'passed': False}
        result['world_sha256'] = hashlib.sha256(world.read_bytes()).hexdigest()
        result['model_sha256'] = {str(p.relative_to(self.out)): hashlib.sha256(p.read_bytes()).hexdigest() for p in (self.out/'models').rglob('*.sdf')}
        result['controller_sha256'] = {name: hashlib.sha256((REPO/name).read_bytes()).hexdigest() for name in (
            'src/lib/turtle_recovery/TurtleRecovery.hpp', 'src/lib/turtle_recovery/TurtleControl.cpp',
            'src/modules/simulation/gz_bridge/GZMixingInterfaceESC.cpp')}
        try:
            self.gz = subprocess.Popen(['gz', 'sim', '-s', '-r', '-v', '2', str(world)], env=self.env,
                                       stdout=logs[0], stderr=subprocess.STDOUT, start_new_session=True)
            deadline = time.monotonic()+15
            while self.latest_pose is None and time.monotonic() < deadline:
                time.sleep(.1)
            if self.latest_pose is None:
                raise RuntimeError('Gazebo did not publish model ground truth')
            # Start the estimator with the physical crash pose already settled.
            # Teleporting after EKF startup would bypass the IMU gyro rotation.
            time.sleep(3)
            runtime = self.out/'rootfs'
            runtime.mkdir(exist_ok=True)
            env = self.env.copy()
            env.update(PX4_GZ_STANDALONE='1', PX4_GZ_WORLD='turtle', PX4_GZ_MODEL_NAME='x500_0',
                       PX4_SYS_AUTOSTART='4001', PX4_SIM_MODEL='gz_x500', HEADLESS='1')
            self.px4 = subprocess.Popen([str(REPO/'build/px4_sitl_default/bin/px4'), '-d',
                                        str(REPO/'build/px4_sitl_default/etc'), '-w', str(runtime)], env=env,
                                       stdout=logs[1], stderr=subprocess.STDOUT, start_new_session=True)
            deadline = time.monotonic()+20
            while self.heartbeat is None and time.monotonic() < deadline:
                self.run_for(.2)
            if self.heartbeat is None:
                raise RuntimeError('No SITL heartbeat on local UDP 14540')
            radio = threading.Thread(target=self.radio_loop, daemon=True)
            radio.start()
            # Only writes isolated SITL parameters; never sends an arming command.
            params = {'COM_RC_IN_MODE': 0, 'RC_CHAN_CNT': 8, 'RC_MAP_ROLL': 1, 'RC_MAP_PITCH': 2, 'RC_MAP_THROTTLE': 3,
                      'RC_MAP_YAW': 4, 'RC_MAP_FLTMODE': 5, 'RC_MAP_ARM_SW': 0, 'COM_FLTMODE1': 8,
                      'COM_FLTMODE6': 17, 'TRTL_PWR': self.args.power, 'TRTL_PULSE': self.args.pulse,
                      'TRTL_TOUT': self.args.timeout, 'TRTL_LPULSE': self.args.level_pulse, 'TRTL_RATE': self.args.rate, 'SDLOG_MODE': 1}
            # Match existing CA geometry to the stock model's actual rotor positions.
            base = ET.parse(REPO/'Tools/simulation/gz/models/x500_base/model.sdf')
            rotor_positions = []
            for motor in range(4):
                pose = base.find(f'.//link[@name="rotor_{motor}"]/pose').text.split()
                rotor_positions.append([float(pose[0]), float(pose[1])])
                params[f'CA_ROTOR{motor}_PX'] = float(pose[0])
                params[f'CA_ROTOR{motor}_PY'] = -float(pose[1])
            for name, value in params.items():
                self.parameter(name, value)
            self.run_for(4)
            result['initial_ground_truth'] = self.latest_pose
            if guard:
                topics = subprocess.run(['gz', 'topic', '-l'], env=self.env, capture_output=True, text=True, timeout=5).stdout.splitlines()
                result['contact_topics'] = [t for t in topics if 'contacts' in t]
                expected_topics = ['/turtle/guard_contacts']+[f'/turtle/rotor{i}_contacts' for i in range(4)]
                if not all(t in topics for t in expected_topics):
                    raise AssertionError('Not all guard/rotor contact monitors were advertised')
            result['initial_guard_ground_contacts'] = self.guard_contacts
            result['initial_rotor_ground_contacts'] = self.rotor_contacts
            if guard and ((not self.args.upright and (self.contact_messages == 0 or self.guard_contacts == 0)) or self.rotor_contacts):
                raise AssertionError('Guard contact precondition failed: require guard on ground, no rotor ground contact')
            if self.args.upright:
                if not 3 <= self.latest_pose[0] < 90:
                    raise AssertionError('Upright setup did not settle within the 3–90 degree entry range')
            elif self.latest_pose[0] < 100:
                raise AssertionError('Crash did not settle inverted')
            self.parameter('TRTL_EN', 1)
            self.run_for(2)
            result['target_system'] = self.connection.target_system
            self.console('listener input_rc -n 1')
            self.console('listener rc_channels -n 1')
            self.console('manual_control status')
            self.console('rc_update status')
            self.console('listener vehicle_acceleration -n 1')
            self.console('listener manual_control_setpoint -n 1')
            before = self.console('listener turtle_status -n 1')
            match = re.search(r'\btilt: ([\d.]+)', before)
            result['initial_estimator_tilt'] = math.degrees(float(match.group(1))) if match else None
            if match is None or abs(result['initial_estimator_tilt']-self.latest_pose[0]) > 5:
                raise AssertionError('Initial estimator attitude does not match the physical crash')
            self.rows.clear()
            self.motor_rows.clear()
            trigger = time.monotonic()-self.start
            self.mode_channel = 2000
            if self.args.fault != 'none':
                deadline = time.monotonic()+3
                while not any(abs(v) > 1 for v in self.latest_velocity) and time.monotonic() < deadline:
                    self.run_for(.02)
                if not any(abs(v) > 1 for v in self.latest_velocity):
                    raise AssertionError('Fault test did not reach active recovery output')
                result['fault_time'] = time.monotonic()-self.start
                if self.args.fault == 'cancel':
                    self.mode_channel = 1000
                else:
                    self.stop_radio.set()
                self.run_for(3)
            else:
                self.run_for(self.args.timeout+3)
            result['selected_mode'] = self.heartbeat.custom_mode
            if self.args.fault == 'none' and self.heartbeat.custom_mode != 12 << 16:
                raise AssertionError('RC mode switch did not select Turtle')
            status_text = self.console('listener turtle_status -n 1')
            match = re.search(r'\breason: (\d+)', status_text)
            result['stop_reason'] = int(match.group(1)) if match else None
            self.console('listener vehicle_land_detected -n 1')
            driven = [r for r in self.motor_rows if any((v > 1 if self.args.upright else v < -1) for v in r[1:])]
            result['driven_thrust_frames'] = len(driven)
            result['negative_thrust_frames'] = sum(any(v < -1 for v in r[1:]) for r in self.motor_rows)
            result['first_pair'] = [i for i, v in enumerate(driven[0][1:]) if (v > 1 if self.args.upright else v < -1)] if driven else []
            w, x, y, z = result['initial_ground_truth'][4:]
            heights = [2*(x*z-w*y)*px+2*(y*z+w*x)*py for px, py in rotor_positions]
            pairs = [[0, 2], [1, 3], [0, 3], [1, 2]]
            scores = [sum(heights[i] for i in pair) for pair in pairs]
            result['initial_rotor_heights_relative_to_body'] = heights
            result['expected_high_pairs'] = [pair for pair, score in zip(pairs, scores) if score >= max(scores)-.0001]
            result['expected_low_pairs'] = [pair for pair, score in zip(pairs, scores) if score <= min(scores)+.0001]
            result['expected_pairs'] = result['expected_low_pairs' if self.args.upright else 'expected_high_pairs']
            result['minimum_ground_truth_tilt'] = min(r[1] for r in self.rows)
            result['final_ground_truth'] = self.latest_pose
            tail = [r[1] for r in self.rows if r[0] >= self.rows[-1][0]-1]
            result['final_second_tilt_span'] = max(tail)-min(tail)
            tail_rows = [r for r in self.rows if r[0] >= self.rows[-1][0]-1]
            result['final_second_position_span'] = math.sqrt(sum((max(r[i] for r in tail_rows)-min(r[i] for r in tail_rows))**2 for i in (2, 3, 4)))
            result['trigger_time'] = trigger
            result['last_nonzero_time'] = max((r[0] for r in self.motor_rows if any(abs(v) > 1 for v in r[1:])), default=None)
            if not driven:
                raise AssertionError('No recovery motor output was produced')
            if result['first_pair'] not in result['expected_pairs']:
                raise AssertionError('First driven pair did not match the expected physical side')
            if self.args.upright and any(v < -1 for r in self.motor_rows for v in r[1:]):
                raise AssertionError('Upright recovery reversed a motor')
            if any(abs(v) > 1 for v in self.latest_velocity):
                raise AssertionError('Motor commands were not stopped after recovery')
            if self.args.fault != 'none':
                allowed = [7] if self.args.fault == 'cancel' else [2, 7]
                if result['stop_reason'] not in allowed:
                    raise AssertionError('Fault did not cancel the active recovery')
                stop = next(r[0] for r in self.motor_rows if r[0] >= result['fault_time'] and all(abs(v) < 1 for v in r[1:]))
                result['fault_stop_latency_seconds'] = stop-result['fault_time']
                if result['fault_stop_latency_seconds'] > .5:
                    raise AssertionError('Thrust continued for more than 500 ms after the fault')
                if self.args.fault == 'rc-loss':
                    self.stop_radio.clear()
                    threading.Thread(target=self.radio_loop, daemon=True).start()
            elif self.args.expect_timeout:
                if result['stop_reason'] != 6:
                    raise AssertionError('Expected bounded timeout on permanent slope')
            else:
                if self.latest_pose[0] > (1 if self.args.upright else 30):
                    raise AssertionError('Craft did not settle within the 30-degree completion angle')
                if result['final_second_tilt_span'] > 1.5:
                    raise AssertionError('Craft was still rotating at the end of the observation')
                if result['final_second_position_span'] > .05:
                    raise AssertionError('Craft was still translating at the end of the observation')
                if result['stop_reason'] != 5:
                    raise AssertionError('PX4 did not confirm the stable 30-degree completion criterion')
            self.mode_channel = 1000
            self.run_for(1)
            exit_status = self.console('listener turtle_status -n 1')
            result['exit_ready'] = bool(re.search(r'ready_for_arming: (True|true|1)', exit_status))
            if not result['exit_ready']:
                raise AssertionError('Direction restoration did not become ready after mode exit')
            if guard and self.rotor_contacts:
                raise AssertionError('Rotor touched ground despite the assumed protective guard')
            result['passed'] = True
        except Exception as exc:
            result['error'] = str(exc)
        finally:
            self.stop_radio.set()
            for process in (self.px4, self.gz):
                if process and process.poll() is None:
                    os.killpg(process.pid, signal.SIGTERM)
                    try:
                        process.wait(timeout=4)
                    except subprocess.TimeoutExpired:
                        os.killpg(process.pid, signal.SIGKILL)
                        process.wait()
            for log in logs:
                log.close()
            for name, rows, fields in [('ground_truth.csv', self.rows, ['time','tilt','x','y','z','qw','qx','qy','qz','motor0','motor1','motor2','motor3']),
                                       ('motor_commands.csv', self.motor_rows, ['time','motor0','motor1','motor2','motor3'])]:
                with (self.out/name).open('w') as f:
                    writer = csv.writer(f); writer.writerow(fields); writer.writerows(rows)
            result['guard_ground_contacts'] = self.guard_contacts
            result['rotor_ground_contacts'] = self.rotor_contacts
            result['contact_messages'] = self.contact_messages
            (self.out/'result.json').write_text(json.dumps(result, indent=2)+'\n')
            print(json.dumps({k: result.get(k) for k in ('case', 'passed', 'stop_reason', 'final_ground_truth', 'rotor_ground_contacts', 'fault_stop_latency_seconds', 'error')}), flush=True)
        return 0 if result['passed'] else 1


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--upright', action='store_true', help='Start upright on the slope; require normal low-side thrust')
    parser.add_argument('--expect-timeout', action='store_true', help='Accept only bounded timeout, not successful levelling')
    parser.add_argument('--case', default='roll-left')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--slope-roll', type=float, default=10)
    parser.add_argument('--slope-pitch', type=float, default=0)
    parser.add_argument('--yaw', type=float, default=0)
    parser.add_argument('--model', choices=['stock', 'guarded'], default='guarded')
    parser.add_argument('--fault', choices=['none', 'cancel', 'rc-loss'], default='none')
    parser.add_argument('--power', type=float, default=.35)
    parser.add_argument('--rate', type=float, default=4)
    parser.add_argument('--timeout', type=float, default=10)
    parser.add_argument('--level-pulse', type=float, default=5)
    parser.add_argument('--pulse', type=float, default=.5)
    sys.exit(Experiment(parser.parse_args()).run())
