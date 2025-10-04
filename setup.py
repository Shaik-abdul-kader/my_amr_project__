from setuptools import setup

package_name = 'my_amr_project__'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    author='shaik abdul kader',
    author_email='ishaikhafiz@gmail.com',
    description='Mixed ROS2 package with C++ and Python nodes',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'detector = detector:main',  # main() in detector.py
        ],
    },
)

