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
        ],
    },
)
