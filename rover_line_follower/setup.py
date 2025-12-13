from setuptools import find_packages, setup

package_name = 'rover_line_follower'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(include=['rover_line_follower', 'rover_line_follower.*']),
    data_files=[
        # package.xml
        ('share/' + package_name, ['package.xml']),
        # resource 파일과 launch 파일은 CMakeLists.txt에서 설치
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Project1',
    maintainer_email='user@example.com',
    description='line follower node',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # rover_line_follower/line_follower_node.py 안에 main() 있어야 함
            'line_follower_node = rover_line_follower.line_follower_node:main',
            'tune_bev = rover_line_follower.tune_bev:main',
            'hsv_tuner = rover_line_follower.hsv_tuner:main',
            'teleop_keyboard = rover_line_follower.teleop_keyboard:main',
        ],
    },
)
