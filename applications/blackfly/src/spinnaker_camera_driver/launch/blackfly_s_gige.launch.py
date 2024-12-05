# -----------------------------------------------------------------------------
# Copyright 2022 Kevin Janesch
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
#
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration as LaunchConfig
from launch.actions import DeclareLaunchArgument as LaunchArg
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory

camera_params = {
    'debug': False,
    'compute_brightness': False,
    'dump_node_map': False,
    'adjust_timestamp' : LaunchConfig('adjust_timestamp'),
    # set parameters defined in blackfly_s_gige.cfg
    'gain_auto': LaunchConfig('gain_auto'),
    'gain': LaunchConfig('gain'),
    'auto_target_grey_value': LaunchConfig('auto_target_grey_value'),
    'exposure_metering_mode': LaunchConfig('exposure_metering_mode'),
    'exposure_auto': LaunchConfig('exposure_auto'),
    'exposure_time': LaunchConfig('exposure_time'),
    'gamma_enable': LaunchConfig('gamma_enable'),
    'gamma': LaunchConfig('gamma'),
    'pixel_format': LaunchConfig('pixel_format'),
    'auto_exposure_time_upper_limit': LaunchConfig('auto_exposure_time_upper_limit'),
    'auto_exposure_time_lower_limit': LaunchConfig('auto_exposure_time_lower_limit'),
    'auto_gain_upper_limit': LaunchConfig('auto_gain_upper_limit'),
    'auto_gain_lower_limit': LaunchConfig('auto_gain_lower_limit'),
    'auto_exposure_control_priority': LaunchConfig('auto_exposure_control_priority'),
    'device_link_throughput_limit': LaunchConfig('device_link_throughput_limit'),
    'max_datarate_threshold': LaunchConfig('device_link_throughput_limit'), # this mirrors device_link_throughput_limit
    'enable_adaptive_compression': LaunchConfig('enable_adaptive_compression'),
    'image_compression_mode': LaunchConfig('image_compression_mode'),
    'line1_selector': 'Line1',
    'line1_linemode': 'Output',
    'line2_selector': 'Line2',
    'line2_v33enable': True,
    'frame_rate_auto': 'Off',
    'frame_rate': LaunchConfig('frame_rate'),
    'frame_rate_enable': True,
    'trigger_mode': 'Off',
    'chunk_mode_active': True,
    'chunk_selector_frame_id': 'FrameID',
    'chunk_enable_frame_id': True,
    'chunk_selector_exposure_time': 'ExposureTime',
    'chunk_enable_exposure_time': True,
    'chunk_selector_gain': 'Gain',
    'chunk_enable_gain': True,
    'chunk_selector_timestamp': 'Timestamp',
    'chunk_enable_timestamp': True,
    'stream_buffer_handling_mode': LaunchConfig('buffer_handling_mode'),
    'gev_ieee_1588': LaunchConfig('PTP_enable'), # PTP enable
    'gev_ieee_1588_mode': LaunchConfig('PTP_mode'),
    'gev_scps_packet_size': LaunchConfig('gev_scps_packet_size'),
    }


def generate_launch_description():
    """Launch blackfly_s camera node."""
    flir_dir = get_package_share_directory('spinnaker_camera_driver')
    config_dir = flir_dir + '/config/'

    name_arg = LaunchArg('camera_name', default_value='blackfly_s', description='camera name')
    serial_arg = LaunchArg('serial', default_value="'20435008'", description='serial number')
    adjust_timestamp_arg = LaunchArg('adjust_timestamp', default_value="False", description="True|False")
    camerainfo_url_arg = LaunchArg('camerainfo_url', default_value="", description='calibration yaml')
    buffer_handling_mode_arg = LaunchArg('buffer_handling_mode', default_value='NewestOnly', description='NewestOnly|NewestFirst|OldestFirst|OldestFirstOverwrite')
    auto_exposure_time_upper_limit_arg = LaunchArg('auto_exposure_time_upper_limit', default_value='15000.0', description='microseconds')
    auto_exposure_time_lower_limit_arg = LaunchArg('auto_exposure_time_lower_limit', default_value='100.0', description='microseconds')
    auto_gain_upper_limit_arg = LaunchArg('auto_gain_upper_limit', default_value='18.00', description='decibels')
    auto_gain_lower_limit_arg = LaunchArg('auto_gain_lower_limit', default_value='0.0', description='decibels')
    device_link_throughput_limit_arg = LaunchArg('device_link_throughput_limit', default_value='125000000', description='megabytes/second')
    enable_adaptive_compression_arg = LaunchArg('enable_adaptive_compression', default_value='False', description='True|False')
    image_compression_mode_arg = LaunchArg('image_compression_mode', default_value='Lossless', description='Off|Lossless')
    gain_auto_arg = LaunchArg('gain_auto', default_value='Continuous', description='Off|Continuous')
    gain_arg = LaunchArg('gain', default_value='15.0', description='gain in decibels')
    frame_rate_arg = LaunchArg('frame_rate', default_value='30', description='frame rate')
    exposure_auto_arg = LaunchArg('exposure_auto', default_value='Continuous', description='Off|Continuous')
    exposure_time_arg = LaunchArg('exposure_time', default_value='15000.0', description='exposure time')
    auto_target_grey_value_arg = LaunchArg('auto_target_grey_value', default_value='Continuous', description='Off|Continuous')
    exposure_metering_mode_arg = LaunchArg('exposure_metering_mode', default_value='Average', description='Average|Partial|Spot')
    gamma_enable_arg = LaunchArg('gamma_enable', default_value='True', description='False|True')
    gamma_arg = LaunchArg('gamma', default_value='0.80', description='float')
    pixel_format_arg = LaunchArg('pixel_format', default_value='BayerRG8', description='depends on camera sensor')
    PTP_enable_arg = LaunchArg('PTP_enable', default_value='False' , description='True|False')
    PTP_mode_arg = LaunchArg('PTP_mode', default_value='Auto', description='Auto|SlaveOnly')
    auto_exposure_control_priority_arg = LaunchArg('auto_exposure_control_priority', default_value='Gain', description='Gain|ExposureTime')
    gev_scps_packet_size_arg = LaunchArg('gev_scps_packet_size', default_value='1500', description='int')

    node = Node(package='spinnaker_camera_driver',
                executable='camera_driver_node',
                output='screen',
                name=[LaunchConfig('camera_name')],
                parameters=[
                    camera_params,
                    {'parameter_file': config_dir + 'blackfly_s_gige.cfg',
                     'serial_number': [LaunchConfig('serial')],
                     'camerainfo_url': LaunchConfig('camerainfo_url')}],
                remappings=[('~/control', '/exposure_control/control')])

    return LaunchDescription([
        name_arg,
        serial_arg,
        adjust_timestamp_arg,
        buffer_handling_mode_arg,
        auto_exposure_time_upper_limit_arg,
        auto_exposure_time_lower_limit_arg,
        auto_gain_upper_limit_arg,
        auto_gain_lower_limit_arg,
        auto_exposure_control_priority_arg,
        device_link_throughput_limit_arg,
        enable_adaptive_compression_arg,
        image_compression_mode_arg,
        gain_auto_arg,
        gain_arg,
        frame_rate_arg,
        camerainfo_url_arg,
        auto_target_grey_value_arg,
        exposure_metering_mode_arg,
        exposure_auto_arg,
        exposure_time_arg,
        gamma_enable_arg,
        gamma_arg,
        pixel_format_arg,
        PTP_enable_arg,
        PTP_mode_arg,
        gev_scps_packet_size_arg,
        node])
