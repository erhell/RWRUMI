from setuptools import find_packages, setup

package_name = "rwrumi_pose"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="RWR Team 4",
    maintainer_email="noreply@example.com",
    description="AprilTag global and relative pose estimation for the UMI device",
    license="MIT",
    entry_points={"console_scripts": ["tag_localizer_node = rwrumi_pose.tag_localizer_node:main"]},
)
