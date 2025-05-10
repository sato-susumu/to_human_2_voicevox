from setuptools import find_packages, setup

package_name = 'to_human_2_voicevox'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sato-susumu',
    maintainer_email='75652942+sato-susumu@users.noreply.github.com',
    description='Bridge node: subscribes to /to_human and calls /voicevox_ros2/speaker_srv.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'to_human_2_voicevox = to_human_2_voicevox.to_human_2_voicevox_node:main',
        ],
    },
)
