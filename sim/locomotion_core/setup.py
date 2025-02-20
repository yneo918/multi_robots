from setuptools import setup

package_name = 'locomotion_core'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vboxuser',
    maintainer_email='vboxuser@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rover_state_controller = locomotion_core.rover_state_controller:main',
            'movebase_kinematics = locomotion_core.movebase_kinematics:main',
            'rover_enable = locomotion_core.rover_enable:main',
            'cmd_roboteq = locomotion_core.cmd_roboteq_jetson:main',
            'en_service = locomotion_core.enable_srv:main',
        ],
    },
)
