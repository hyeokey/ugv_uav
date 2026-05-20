from glob import glob
from setuptools import setup

package_name = "ugv_description"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (f"share/{package_name}/urdf", glob("urdf/*.urdf")),
        (f"share/{package_name}/rviz", glob("rviz/*.rviz")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="dong",
    maintainer_email="dong@example.com",
    description="URDF and TF helpers for the UGV in the UAV/UGV docking simulation.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "aruco_detector_node = ugv_description.aruco_detector_node:main",
            "aruco_precision_landing_node = ugv_description.aruco_precision_landing_node:main",
            "lidar_local_planner_node = ugv_description.lidar_local_planner_node:main",
            "px4_odometry_tf_node = ugv_description.px4_odometry_tf_node:main",
            "laser_scan_to_pointcloud_node = ugv_description.laser_scan_to_pointcloud_node:main",
            "lidar_scan_debug_node = ugv_description.lidar_scan_debug_node:main",
        ],
    },
)
