#!/usr/bin/env python3
"""Offline: fold pinned YOLOX Focus into an equivalent 6x6 convolution for NCNN."""
import argparse
import hashlib
from pathlib import Path
import subprocess


def fold_focus_weights(original):
    import numpy as np
    if original.shape != (16, 12, 3, 3):
        raise ValueError('Unexpected Focus weights')
    folded = np.zeros((16, 3, 6, 6), dtype=np.float32)
    for block, (dy, dx) in enumerate(((0,0), (1,0), (0,1), (1,1))):
        folded[:, :, dy::2, dx::2] = original[:, block*3:(block+1)*3]
    return folded


def convert(source, destination, pnnx):
    import numpy as np
    import onnx
    from onnx import numpy_helper, helper
    from person_watch import MODEL_SHA256
    if hashlib.sha256(Path(source).read_bytes()).hexdigest() != MODEL_SHA256:
        raise ValueError('Unexpected source model')
    model = onnx.load(source)
    nodes = list(model.graph.node)
    if [n.op_type for n in nodes[:10]] != ['Slice']*8 + ['Concat', 'Conv']:
        raise ValueError('Unexpected Focus graph')
    first_conv = nodes[9]
    weights = next(i for i in model.graph.initializer if i.name == first_conv.input[1])
    original = numpy_helper.to_array(weights)
    # Original concatenation: even/even, odd/even, even/odd, odd/odd.
    folded = fold_focus_weights(original)
    weights.CopyFrom(numpy_helper.from_array(folded, weights.name))
    first_conv.input[0] = model.graph.input[0].name
    del first_conv.attribute[:]
    first_conv.attribute.extend([helper.make_attribute('kernel_shape',[6,6]),
                                 helper.make_attribute('strides',[2,2]),
                                 helper.make_attribute('pads',[2,2,2,2]),
                                 helper.make_attribute('group',1),
                                 helper.make_attribute('dilations',[1,1])])
    del model.graph.node[:]
    model.graph.node.extend(nodes[9:])
    onnx.checker.check_model(model)
    onnx.save(model, destination)
    subprocess.run([pnnx, str(destination), 'inputshape=[1,3,416,416]', 'fp16=0'], check=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--source', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--pnnx', required=True)
    args = parser.parse_args()
    convert(args.source, args.output, args.pnnx)
