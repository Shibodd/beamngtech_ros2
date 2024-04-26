import queue
import threading
import rosbags.rosbag2 as rb2

class BagWriter:
  def __init__(self, path: str, sensor_topics: dict, typestore):
    self.path = path
    self.sensor_topics = sensor_topics
    self.typestore = typestore
    self.idle = None

  def add_msgs(self, sensor_data):
    self.msg_queue.put(sensor_data)

  def start(self):
    del self.idle
    self.stop_requested = False
    self.msg_queue = queue.Queue()
    self.thread = threading.Thread(target=self._work)
    self.connections = {}
    self.thread.start()

  def __enter__(self):
    self.start()
    return self

  def __exit__(self, *args, **kwargs):
    self.stop_requested = True
    self.thread.join()

  def _work(self):
    with rb2.Writer(self.path) as self.writer:
      while not self.stop_requested:
        while True:
          if self.stop_requested:
            return
          try:
            sensor_data = self.msg_queue.get(timeout=0.5)
            break
          except queue.Empty:
            pass
        for sensor_name, msgs in sensor_data.items():
          self._write_msgs(sensor_name, msgs)

  def _get_conn(self, sensor_name, msgs):
    if not sensor_name in self.connections:
      self.connections[sensor_name] = self.writer.add_connection(self.sensor_topics[sensor_name][0], msgs[0][1].__msgtype__, typestore=self.typestore, offered_qos_profiles=self.sensor_topics[sensor_name][1])
    return self.connections[sensor_name]

  def _write_msgs(self, sensor_name, msgs):
    if not msgs:
      return
    conn = self._get_conn(sensor_name, msgs)
    for ns, msg in msgs:
      print(ns / 10**9, msg.__msgtype__)
      self.writer.write(conn, ns, self.typestore.serialize_cdr(msg, msg.__msgtype__))
