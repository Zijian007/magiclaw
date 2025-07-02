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
from magiclaw.config import FingerNetConfig

finger_net = FingerNet(
    fingernet_cfg=FingerNetConfig(
        name="FingerNet",
        model_path="<model_path>",
        device="auto"
    )
)
```

where `<model_path>` is the path of the ONNX model file.

To run inference, you can use the `infer` method of the `FingerNet` class.

```
force, node = finger_net.infer(motion)
```

where `motion` (np.ndarray) is the input motion data, and `force` (np.ndarray) and `node` (np.ndarray) are the 
output force on the bottom surface and the node displacement of the finger, respectively.

For more information, please refer to https://github.com/asMagiClaw/meta-finger
"""

from .fingernet import FingerNet