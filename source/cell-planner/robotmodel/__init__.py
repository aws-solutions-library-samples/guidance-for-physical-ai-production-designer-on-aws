"""Robot models read from URDF files (docs/p3_urdf.md). ``URDFRobot`` is the single source of joint kinematics and
limits for env/, trajgen/ and the policies; ``robotmodel.override`` points a whole process at another URDF / base."""
from .urdf_robot import ACCEL_DEFAULTS, URDFRobot, compare_limits, parse_urdf  # noqa: F401
