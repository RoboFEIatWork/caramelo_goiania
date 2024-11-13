from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Argumentos de lançamento
    configuration_directory = LaunchConfiguration('configuration_directory', default='config')
    configuration_basename = LaunchConfiguration('configuration_basename', default='my_lidar_config.lua')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # Nó Cartographer
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '-configuration_directory', configuration_directory,
            '-configuration_basename', configuration_basename
        ],
        remappings=[
            ('/scan', '/meu_lidar_scan')  # Remapeando o tópico do LiDAR
        ]
    )

    # Descrição do Lançamento
    return LaunchDescription([
        DeclareLaunchArgument(
            'configuration_directory',
            default_value='/home/victor/caramelo_goiania/src/navegacao_pkg/navegacao_pkg/config',
            description='Diretório de configuração do Cartographer'
        ),
        DeclareLaunchArgument(
            'configuration_basename',
            default_value='my_lidar_config.lua',
            description='Arquivo de configuração do Cartographer'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Usar tempo de simulação (defina como false para LiDAR real)'
        ),
        cartographer_node
    ])
