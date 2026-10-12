"""GPU acceptance checks, run inside each image by jetson_gpu_smoke.sh.

No cameras, network, models downloaded at runtime, or actuators are needed.
The nvblox component must actually load; a live ROS launch process alone is
not sufficient evidence that its GPU node initialized successfully.
"""
import argparse
import ctypes
import subprocess


def check_torch(app):
    import torch
    import torchvision

    assert torch.version.cuda, 'CPU-only PyTorch was installed'
    assert torch.cuda.is_available(), 'CUDA is unavailable in the container'
    print(f'{torch.__version__=}, {torch.version.cuda=}, '
          f'GPU={torch.cuda.get_device_name(0)}', flush=True)
    # cuBLAS, cuDNN, and torchvision's native CUDA extension, not just
    # driver discovery. Incompatible GPU architectures fail here.
    x = torch.ones((64, 64), device='cuda')
    assert torch.allclose(x @ x, torch.full_like(x, 64))
    conv = torch.nn.Conv2d(3, 8, 3).cuda()
    assert conv(torch.ones((1, 3, 32, 32), device='cuda')).isfinite().all()
    boxes = torch.tensor([[0., 0., 10., 10.], [0., 0., 10., 10.]], device='cuda')
    scores = torch.tensor([0.9, 0.8], device='cuda')
    assert torchvision.ops.nms(boxes, scores, 0.5).tolist() == [0]
    if app == 'depthanything':
        from depthanything_pkg.metric_depth import load_model
        # Match the node's default outdoor metric checkpoint.
        model, _ = load_model('vits', 'vkitti', 80.0, device='cuda')
        import numpy as np
        with torch.inference_mode():
            depth = model.infer_image(np.zeros((64, 64, 3), dtype=np.uint8), 56)
        assert np.isfinite(depth).all()
    else:
        from ultralytics import YOLO
        # A packaged architecture exercises Ultralytics without downloading
        # weights. Detection quality is tested separately with real frames.
        model = YOLO('yolo11n.yaml')
        model.predict(source=__import__('numpy').zeros((64, 64, 3), dtype='uint8'),
                      device=0, imgsz=64, verbose=False)
    torch.cuda.synchronize()


def check_nvblox():
    driver = ctypes.CDLL('libcuda.so.1')
    assert driver.cuInit(0) == 0, 'CUDA driver initialization failed'
    count = ctypes.c_int()
    assert driver.cuDeviceGetCount(ctypes.byref(count)) == 0 and count.value > 0

    import rclpy
    from composition_interfaces.srv import LoadNode
    from ament_index_python.packages import get_package_prefix
    from pathlib import Path

    # Nav2 must have its native nvblox costmap plugin too.
    plugin_dir = Path(get_package_prefix('nvblox_nav2')) / 'lib'
    plugins = list(plugin_dir.glob('libnvblox_nav2.so'))
    assert plugins, f'No nvblox costmap plugin in {plugin_dir}'
    for plugin in plugins:
        ctypes.CDLL(str(plugin))

    process = subprocess.Popen([
        'ros2', 'run', 'rclcpp_components', 'component_container_mt',
        '--ros-args', '-r', '__node:=jetson_gpu_smoke_container',
    ], start_new_session=True)
    rclpy.init()
    node = rclpy.create_node('jetson_gpu_smoke_client')
    try:
        client = node.create_client(LoadNode, '/jetson_gpu_smoke_container/_container/load_node')
        assert client.wait_for_service(timeout_sec=30), 'Component manager did not start'
        request = LoadNode.Request()
        request.package_name = 'nvblox_ros'
        request.plugin_name = 'nvblox::NvbloxNode'
        request.node_name = 'jetson_gpu_smoke_nvblox'
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=90)
        assert future.done(), 'nvblox component load timed out'
        response = future.result()
        assert response.success, response.error_message
        assert process.poll() is None, 'nvblox component container exited'
    finally:
        import os
        import signal
        node.destroy_node()
        rclpy.shutdown()
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('app', choices=('nvblox', 'pantiltros', 'depthanything'))
    args = parser.parse_args()
    if args.app == 'nvblox':
        check_nvblox()
    else:
        check_torch(args.app)
    print(f'{args.app}: GPU smoke check passed', flush=True)


if __name__ == '__main__':
    main()
