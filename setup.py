from setuptools import find_packages, setup

package_name = 'text_detect'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Creador270',
    maintainer_email='creadorjp@gmail.com',
    description='A simple node that detects text in an image',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'text_detect = text_detect.text_detect:main'
        ],
    },
)
