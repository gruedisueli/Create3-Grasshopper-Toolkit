from setuptools import setup

package_name = 'droids_controller'

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
    maintainer='gruedisueli',
    maintainer_email='',
    description='A simple toolkit for controlling multiple robots in a single unified scene.',
    license='see license file',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "example_scene = droids_controller.example_scene:main",
            "drive_straight = droids_controller.drive_straight:main",
            "song_drive_straight_rotate = droids_controller.song_drive_straight_rotate:main",
            "drive_arc = droids_controller.drive_arc:main",
            "rotate = droids_controller.rotate:main",
            "play_song = droids_controller.play_song:main",
            "multi_segment_path = droids_controller.multi_segment_path:main",
            "multiple_robots_2 = droids_controller.multiple_robots_2:main",
            "multiple_robots_3 = droids_controller.multiple_robots_3:main"
        ],
    },
)
