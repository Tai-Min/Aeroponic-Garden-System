from setuptools import setup

package_name = 'tank_control'
submodules = 'tank_control/submodules'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, submodules],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', "launch/system.launch.py"]),
        ('share/' + package_name + "/config/", ["config/params.yaml"]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='arane',
    maintainer_email='arane@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'level_observer = tank_control.level_observer:main',
            'uc_bridge = tank_control.uc_bridge:main',
            'pump_driver = tank_control.pump_driver:main',
            'quality_observer = tank_control.quality_observer:main'
        ],
    },
)
