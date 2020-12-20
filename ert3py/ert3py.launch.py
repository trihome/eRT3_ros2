# -----------------------------------------------
# ROS LAUNCH
#
# The MIT License (MIT)
# Copyright (C) 2020 myasu.
# -----------------------------------------------

"""Launch a add_two_ints_server and a (synchronous) add_two_ints_client."""

import launch
import launch_ros.actions


def generate_launch_description():

    ### ここにlaunchしたいノードを定義
    ### node_executableのところは、setup.pyのなかの
    ### entry_pointsで指定した （例）pub = node.Pub:main
    ### の左辺側の文字と合わせて下さい
    pub = launch_ros.actions.Node(
        package='ert3py', node_executable='pub', output='screen')
    sub = launch_ros.actions.Node(
        package='ert3py', node_executable='sub', output='screen')

    ### こちらにもlaunchしたいノードを記述
    ### 上記で定義した （例）pub = launch_ros.actions.Node
    ### の、左辺側の変数を列挙します。
    ### 記述しなかったら、そのノードは起動しません。
    ###
    ### target_action=sub に記述したノードが落ちたら、
    ### launchで起動したものが一式落ちます。
    return launch.LaunchDescription([
        pub,
        sub,
        # TODO(wjwwood): replace this with a `required=True|False` option on ExecuteProcess().
        # Shutdown launch when client exits.
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=sub,
                on_exit=[launch.actions.EmitEvent(
                    event=launch.events.Shutdown())],
            )),
    ])