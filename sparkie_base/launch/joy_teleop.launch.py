import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Percorso del file di configurazione per teleop_twist_joy
    # Useremo parametri direttamente nel launch file, ma per file YAML esterni il percorso sarebbe qui.
    # teleop_config_path = os.path.join(
    #    get_package_share_directory('my_xbox_teleop'),
    #    'config',
    #    'xbox_one_teleop.yaml'
    # )

    # Nodo per il joy_node
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{
            'dev': '/dev/input/js0',  # Dispositivo joystick, verifica che sia corretto
            'deadzone': 0.12,          # Zona morta per evitare drift dello stick
            'autorepeat_rate': 20.0,  # Frequenza di pubblicazione quando non ci sono cambiamenti
        }]
    )

    # Nodo per teleop_twist_joy
    teleop_twist_joy_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        remappings=[
            ('/cmd_vel', '/sparkie/board/cmd_vel') # Remappa /cmd_vel al topic desiderato
        ],
        parameters=[{
            # Configurazione del controller Xbox Series S/One
            # Assi:
            # 0: Left Stick X (sinistra/destra)
            # 1: Left Stick Y (avanti/indietro)
            # 2: Right Stick X (sinistra/destra)
            # 3: Right Stick Y (avanti/indietro)
            # 4: Left Trigger (asse 0 a 1)
            # 5: Right Trigger (asse 0 a 1)
            # 6: D-Pad X (sinistra/destra)
            # 7: D-Pad Y (su/giù)

            # Pulsanti:
            # 0: A
            # 1: B
            # 2: X
            # 3: Y
            # 4: LB
            # 5: RB
            # 6: View/Back
            # 7: Menu/Start
            # 8: Xbox Button
            # 9: Left Stick Click
            # 10: Right Stick Click

            'axis_linear.x': 1,           # Left Stick Y per il movimento lineare avanti/indietro
            'scale_linear.x': 0.2,        # Scala per la velocità lineare (max 0.5 m/s)
            'scale_linear_turbo.x': 0.3,  # Scala per la velocità lineare turbo (max 1.0 m/s)

            'axis_angular.yaw': 2,          # Right Stick X per il movimento angolare sinistra/destra
            'scale_angular.yaw': 1.0,       # Scala per la velocità angolare (max 1.0 rad/s)
            'scale_angular_turbo.yaw': 1.5, # Scala per la velocità angolare turbo (max 1.5 rad/s)

            'enable_button': 6,           # Pulsante LB per abilitare il teleoperazione (deadman switch)
            'enable_turbo_button': 7 ,     # Pulsante RB per attivare la modalità turbo (velocità più alte)

            # È possibile configurare più assi, es. per movimento laterale o altezza se il robot lo supporta
            # 'axis_linear.y': <un altro asse>,
            # 'axis_angular.x': <un altro asse>,
            # 'axis_angular.y': <un altro asse>,
        }]
    )

    return LaunchDescription([
        joy_node,
        teleop_twist_joy_node
    ])
