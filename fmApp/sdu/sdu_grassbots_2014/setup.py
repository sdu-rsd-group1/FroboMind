from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
  scripts=[
  'mission/keyboard_mission.py',
  'mission/wiimote_mission.py',
  'route/route_socketd.py',
  'navigation/area_coverage_casmo_node.py',
  'navigation/area_coverage_casmo_implement_node.py',
  'monitor/show_navigation_status_node.py'],
)

setup(**d)
