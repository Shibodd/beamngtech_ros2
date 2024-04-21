import beamngpy.beamng

BEAMNG_HOME = "F:\BeamNG.tech.v0.31.3.0"

beamngpy.beamng.beamng.sleep = exit # read BeamNGpy.open
with beamngpy.BeamNGpy(host="localhost", port=64256, home=BEAMNG_HOME, quit_on_close=False):
  pass