from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'rf_source'
package_dir = os.path.join(os.path.dirname(__file__), package_name)

# Create console_scripts dynamically
console_scripts_list = []
for fname in os.listdir(package_dir):
    if fname.endswith('.py') and fname != '__init__.py':
        module_name = fname[:-3]  # remove .py
        console_scripts_list.append(
            f"{module_name} = {package_name}.{module_name}:main"
        )

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*'))
    ],
    install_requires=['setuptools', 'digi-xbee'],
    zip_safe=True,
    maintainer='christian',
    maintainer_email='cpedrigal@scu.edu',
    description='RF Source for Adaptive Navigation',
    license='ECL-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': console_scripts_list
    },
)
