from setuptools import find_packages, setup

package_name = 'navegacao_pkg'

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
    maintainer='atwork',
    maintainer_email='robofei.atwork@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'serial_node_arduino = navegacao_pkg.serial_node_arduino:main',
            'serial_node_isaac = navegacao_pkg.serial_node_isaac:main',
            'segue_linha = navegacao_pkg.segue_linha:main',
            'camera_node = navegacao_pkg.camera_node:main',
        ],
    },
)
