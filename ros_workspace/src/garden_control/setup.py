from setuptools import setup

package_name = 'garden_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', "launch/system.launch.py"]),
        ('share/' + package_name + "/config/", ["config/params.yaml"]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='arane',
    maintainer_email='Tai-Min@users.noreply.github.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'zigbee_bridge = garden_control.zigbee_bridge:main'
        ],
    },
)
