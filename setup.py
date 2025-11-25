from setuptools import find_packages, setup

package_name = 'enme480_project'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='enme480_docker',
    maintainer_email='enme480_docker@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'main_pipeline = enme480_project.main_pipeline:main',
            'aruco_tracker = enme480_project.block_detection_aruco:main',
            'perspective_gazebo = enme480_project.perspective_gazebo:main'
        ],
    },
)
