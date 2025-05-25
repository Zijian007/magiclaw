#!/usr/bin/env python

"""
FingerNet
=========

FingerNet is a neural network model for infering the proprioception of the soft finger.
The input is the motion, and the output is the force on the bottom surface and the displacement of mesh nodes.
This module is to deploy the model using ONNX runtime.

Usage
-----------------

To use the FingerNet model, you can initialize the FingerNet class by:

```
from magiclaw.models.fingernet import FingerNet

finger_net = FingerNet(
    name="FingerNet",
    model_path=<path_to_your_model>,
    device="auto"
)
```

where `<path_to_your_model>` is the path to your ONNX model file.

To run inference, you can use the `infer` method of the `FingerNet` class.

```
force, node = finger_net.infer(motion)
```

where `motion` (np.ndarray) is the input motion data, and `force` (np.ndarray) and `node` (np.ndarray) are the 
output force on the bottom surface and the node displacement of the finger, respectively.

For more information, please refer to https://github.com/asMagiClaw/metafinger
"""

from .fingernet import FingerNet