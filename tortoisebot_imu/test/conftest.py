"""Make the tests runnable from the repository root as well as from here.

The ROS package directory and the Python package inside it share the name
`tortoisebot_imu`, so running pytest from the repo root resolves the
import to the outer directory. Putting the package root on sys.path removes
the ambiguity wherever pytest is started.
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
