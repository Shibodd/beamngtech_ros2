from dataclasses import dataclass
import numpy as np

import beamngpy
import beamngpy.sensors
import utils.geometry_helpers as geometry_helpers
from .factories import factory

FACTORY_MAP = {}

class ClassicSensorBase:
  def __init__(self, node):
    self.vehicle = node.vehicle
    self.vehicle: beamngpy.Vehicle
    
    if self.NAME not in self.vehicle.sensors._sensors:
      self.vehicle.sensors.attach(self.NAME, self.TYPE())

  def poll(self, workspace):
    self.parse(workspace, self.vehicle.sensors.data[self.NAME])

@factory('state', FACTORY_MAP)
class StateSensor(ClassicSensorBase):
  NAME = 'state'
  TYPE = beamngpy.sensors.State

  def parse(self, workspace, sample):
    workspace[StateSensor.NAME] = [{
      'time': workspace['timer'],
      'position': np.array(sample['pos'], dtype=np.float64).reshape((3,)),
      'orientation': geometry_helpers.quat_from_fwd_up(sample['dir'], sample['up']),
      'velocity': np.array(sample['vel'], dtype=np.float64).reshape((3,))
    }]

@factory('electrics', FACTORY_MAP)
class ElectricsSensor(ClassicSensorBase):
  NAME = 'electrics'
  TYPE = beamngpy.sensors.Electrics

  def parse(self, workspace, sample):
    sample['time'] = workspace['timer']
    workspace[ElectricsSensor.NAME] = [sample]