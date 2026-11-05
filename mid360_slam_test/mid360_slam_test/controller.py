"""Second-terminal operator menu. Commands are acknowledged and idempotent."""
import json
import math
import threading
import time
import uuid
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from .nodes import latched


class Controller(Node):
    def __init__(self):
        super().__init__('test_controller')
        self.state, self.acks = {}, {}
        self.pub = self.create_publisher(String, '/slam_test/commands', 10)
        self.create_subscription(String, '/slam_test/test_state', self.receive, latched())
        self.create_subscription(String, '/slam_test/command_ack', self.ack, 20)

    def receive(self,msg):
        self.state = json.loads(msg.data)

    def ack(self,msg):
        data = json.loads(msg.data)
        self.acks[data['id']] = data

    def send(self,action,**kwargs):
        command = dict(id=uuid.uuid4().hex,action=action,**kwargs)
        for attempt in range(3):
            self.pub.publish(String(data=json.dumps(command,allow_nan=False)))
            deadline = time.monotonic()+2.
            while time.monotonic() < deadline and rclpy.ok():
                result = self.acks.pop(command['id'],None)
                if result:
                    print(('DITERIMA: ' if result['ok'] else 'DITOLAK: ')+result['reason'])
                    return result
                time.sleep(.05)
        print('Tidak ada ACK. Jangan mulai gerakan; cek evaluator dan domain ROS.')
        return None


def main(args=None):
    rclpy.init(args=args)
    node = Controller()
    worker = threading.Thread(target=lambda:rclpy.spin(node),daemon=True)
    worker.start()
    print('KONTROL TES MANUAL — tidak mengirim perintah ke drone.')
    print('Tunggu tracking sehat. Tetapkan penanda fisik dan referensi sebelum mulai.')
    try:
        while rclpy.ok():
            if not node.state:
                print('Menunggu /slam_test/test_state ...')
                time.sleep(1.)
                continue
            s = node.state
            print('\nTahap:',s['stage'],'| Fase:',s['phase'],'| Siap mulai:',s['can_start'])
            print('Referensi:',json.dumps(s['reference']))
            if s['phase'] == 'COMPLETE':
                print('Sesi selesai. Hentikan launch dengan Ctrl+C agar recorder menutup bag.')
                break
            if s['phase'] == 'REVIEW' and s['results']:
                print('HASIL:',json.dumps(s['results'][-1],indent=2))
            choice = input('[Enter] mulai/selesai | [e] edit referensi | [n] lanjut | '
                           '[r] ulangi | [s] lewati | [p] perbarui | [q] akhiri: ').strip().lower()
            if choice == 'q':
                node.send('quit')
                break
            if choice == 'p':
                continue
            if choice == 'e':
                if s['phase'] != 'READY':
                    print('Referensi hanya dapat diubah sebelum mulai.')
                    continue
                kind = s['reference']['kind']
                try:
                    if kind == 'stationary':
                        value = float(input('Durasi diam (detik): '))
                        node.send('reference',duration_s=value)
                    elif kind == 'rotation':
                        value = float(input('Sudut bertanda (+360 kiri, -360 kanan): '))
                        node.send('reference',reference_yaw_deg=value)
                    elif kind == 'translation':
                        values = [float(v) for v in input('Perpindahan x y z (m), sumbu sensor saat mulai: ').split()]
                        if len(values)!=3 or not all(math.isfinite(v) for v in values):
                            raise ValueError('masukkan tiga angka finite')
                        node.send('reference',reference_xyz_m=values)
                    else:
                        print('Loop: kembali ke posisi DAN orientasi awal; referensi nol.')
                except ValueError as exc:
                    print('Input tidak valid:',exc)
                continue
            actions = {'n':'next','r':'repeat','s':'skip'}
            if choice in actions:
                node.send(actions[choice])
            elif choice == '':
                if s['phase'] == 'READY':
                    if not s['can_start']:
                        print('Belum siap: tracking harus sehat dan perangkat diam pada penanda awal.')
                        continue
                    answer = input('Referensi sudah diukur, perangkat diam dan operator siap? ketik ya: ')
                    if answer.strip().lower() == 'ya':
                        node.send('start',reference_confirmed=True)
                elif s['phase'] == 'RUNNING':
                    if s['reference']['kind'] == 'stationary':
                        print('Tes diam berakhir otomatis setelah durasi terpenuhi.')
                    else:
                        node.send('finish')
                else:
                    print('Tunggu hasil atau pilih perintah sesuai fase.')
    except (KeyboardInterrupt,EOFError):
        print('\nMenu ditutup; tahap aktif belum otomatis selesai. Buka menu lagi atau hentikan launch.')
    finally:
        rclpy.shutdown()
        worker.join(timeout=2.)
        node.destroy_node()
