from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
  scripts=[
  'mission/simple_mission.py',
  'mission/keyboard_mission.py',
  'monitor/plot_propulsion_feedback_node.py',
  'navigation/area_coverage_casmo_node.py',
  'navigation/area_coverage_casmo_implement_node.py',
  'monitor/robot_track_map_node.py'],
)

setup(**d)


