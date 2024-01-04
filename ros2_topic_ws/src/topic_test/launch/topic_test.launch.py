from launch import LaunchDescription
from launch_ros.actions import Node
 
def generate_launch_description():
    return LaunchDescription([
        Node(
            package='topic_test',           # 节点所在的功能包
            namespace='',                   # 节点所在的命名空间
            executable='publisher',   # 节点的可执行文件名
            name='topic_publisher'                         # 对节点重新命名
        ),
        Node(
            package='topic_test',
            namespace='',
            executable='subscriber',
            name='topic_subscriber'
        )
    ])