"""NCNN Vulkan GPU runner, single CPU thread, pinned converted YOLOX weights."""
import hashlib
from pathlib import Path

PARAM_SHA256 = '89ae9f7c5512cb4b4fb96c9188b7761f9dbfefb5cfbac35cf6968e1ce582b121'
WEIGHTS_SHA256 = '9274d3ca586f76fb9d7493da199190e55c584035166724f6cfd294fd02aa7f60'


class NcnnRunner:
    def __init__(self, param, weights):
        import ncnn
        import numpy as np
        self.ncnn, self.np = ncnn, np
        for filename, expected in ((param, PARAM_SHA256), (weights, WEIGHTS_SHA256)):
            if hashlib.sha256(Path(filename).read_bytes()).hexdigest() != expected:
                raise ValueError('Converted model checksum mismatch')
        if ncnn.get_gpu_count() < 1:
            raise RuntimeError('Vulkan GPU unavailable; CPU fallback is disabled')
        self.net = ncnn.Net()
        self.net.set_vulkan_device(0)
        self.net.opt.use_vulkan_compute = True
        self.net.opt.num_threads = 1
        self.net.opt.use_fp16_arithmetic = False
        self.net.opt.use_fp16_storage = False
        self.net.opt.use_fp16_packed = False
        if self.net.load_param(param) != 0 or self.net.load_model(weights) != 0:
            raise RuntimeError('NCNN model load failed')

    def forward(self, image):
        with self.net.create_extractor() as extractor:
            # Transposed CHW arrays may retain non-contiguous strides. NCNN's
            # buffer binding requires a contiguous array, not just float32 dtype.
            blob = self.ncnn.Mat(self.np.ascontiguousarray(image[0])).clone()
            if extractor.input('in0', blob) != 0:
                raise RuntimeError('NCNN input failed')
            result, output = extractor.extract('out0')
            if result != 0:
                raise RuntimeError('NCNN inference failed')
            array = self.np.array(output).copy()
            if array.shape != (3549, 85) or not self.np.isfinite(array).all():
                raise RuntimeError('Invalid NCNN output')
            return array[None]

    def close(self):
        self.net.clear()
