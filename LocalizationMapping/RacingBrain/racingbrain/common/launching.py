from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def include_launch(package_name, launch_file, launch_arguments=None, condition=None):
    package_share = get_package_share_directory(package_name)
    launch_path = f"{package_share}/launch/{launch_file}"
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_path),
        launch_arguments=(launch_arguments or {}).items(),
        condition=condition,
    )
