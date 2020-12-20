from setuptools import find_packages
from setuptools import setup
import os
from glob import glob

### ここにパッケージ名を記述 ###
package_name = 'ert3py'

### 必要に応じてパッケージ情報を記述 ###
setup(
    name=package_name,
    version='0.7.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='ubuntu',
    author_email='ubuntu@todo.todo',
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: MIT License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description=(
        'pubsubpy sample'
    ),
    license='MIT License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pub = node.Pub:main',
            'sub = node.Sub:main',
        ],
    },
)
### ↑ entry_pointsの部分に、パッケージに含まれるノード(pythonソース)を列挙 ###