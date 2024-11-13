from setuptools import find_packages, setup

package_name = 'navegacao_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # ('share/' + package_name + '/launch', ['launch/cartographer.launch.py']),
        # ('share/' + package_name + '/navegacao_pkg/config', ['config/my_lidar_config.lua']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='atwork',
    maintainer_email='robofei.atwork@gmail.com',
    description='Pacote de navegação para mapeamento e controle do robô',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'serial_node_arduino = navegacao_pkg.serial_node_arduino:main',
            'segue_linha = navegacao_pkg.segue_linha:main',
            'camera_node = navegacao_pkg.camera_node:main',
        ],
    },
)
