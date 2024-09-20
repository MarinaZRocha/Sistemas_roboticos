from setuptools import find_packages, setup

package_name = 'test_pkg'

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
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "py_node = test_pkg.node1:main",
            "publisher = test_pkg.publisher:main",
            "MoveTurtle = test_pkg.just_target:main",
            "target2 = test_pkg.target2:main",
            "go_target = test_pkg.go_target:main",
            "test = test_pkg.test:main",
        ],
    },
)
