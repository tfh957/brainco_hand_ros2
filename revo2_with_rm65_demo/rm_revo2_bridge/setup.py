from setuptools import find_packages, setup

package_name = 'rm_revo2_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(include=(package_name, f'{package_name}.*')),
    data_files=[
        (f'share/{package_name}', ['package.xml']),
        (f'share/{package_name}/launch', ['launch/rm_revo2_bridge.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fishros',
    maintainer_email='fishros@localhost',
    description='RM65/Revo2 bridge package',
    license='Apache-2.0',
)
