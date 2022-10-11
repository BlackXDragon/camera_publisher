from setuptools import setup
import glob

package_name = 'camera_publisher'

resources = glob.glob("resource/*yaml")
resources.extend(glob.glob("resource/*.png"))

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + 'resources', resources),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='george',
    maintainer_email='george@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'image_publisher = camera_publisher.image_publisher:main',
            'camera_publisher = camera_publisher.camera_publisher:main'
        ],
    },
)
