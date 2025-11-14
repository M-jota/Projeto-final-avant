from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([

        # --- Nó da Visão ---
        Node(
            package='visao_pkg',
            executable='node_visao',
            name='node_visao_desafio',
            output='screen',
            parameters=[]
        ),

        # --- Nó da Navegação ---
        Node(
            package='navegacao_pkg',
            executable='node_navegacao',
            name='navegacao_simples_node',
            output='screen',
            parameters=[]
        )

    ])
