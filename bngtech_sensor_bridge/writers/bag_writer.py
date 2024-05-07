import queue
import threading
import rosbags.rosbag2 as rb2

class BagWriter:
  def __init__(self, path: str, sensor_topics: dict, typestore, error_handler):
    self.path = path
    self.sensor_topics = sensor_topics
    self.typestore = typestore
    self.idle = None
    self.error_handler = error_handler

  def add_data(self, sensor_data):
    self.queue.put(sensor_data)

  def start(self):
    del self.idle
    self.stop_requested = False
    self.queue = queue.Queue()
    self.thread = threading.Thread(target=self._work)
    self.connections = {}
    self.thread.start()

  def stop(self, *args, **kwargs):
    self.stop_requested = True
    self.thread.join()

  def _work(self):
    try:
      with rb2.Writer(self.path) as self.writer:
        while not self.stop_requested:
          while True:
            if self.stop_requested:
              return
            try:
              sensor_data = self.queue.get(timeout=0.5)
              break
            except queue.Empty:
              pass
          for sensor_name, data in sensor_data.items():
            self._write_data(sensor_name, data)
    except Exception as e:
      self.error_handler(e)

  def _get_conn(self, sensor_name, msg_type):
    if not sensor_name in self.connections:
      self.connections[sensor_name] = self.writer.add_connection(self.sensor_topics[sensor_name][0], msg_type, typestore=self.typestore, offered_qos_profiles=self.sensor_topics[sensor_name][1])
    return self.connections[sensor_name]

  def _write_data(self, sensor_name, data):
    if not data:
      return
    
    for sample in data:
      ns, msg = sample.to_msg(self.typestore)
      print(sample.time, msg.__msgtype__)

      conn = self._get_conn(sensor_name, msg.__msgtype__)
      self.writer.write(conn, ns, self.typestore.serialize_cdr(msg, msg.__msgtype__))
