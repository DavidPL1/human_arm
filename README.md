Human Arm Model
===============

This package is derived from the [human_hand_description](https://github.com/ubi-agni/human_hand) package (which in turn is based on [pisa_hand_description](https://github.com/WEARHAP/hand-models/tree/master/pisa_hand_description)).


This package models the human arm starting from the elbow joint.
Thumb joint limits inspired by: https://www.ncbi.nlm.nih.gov/pmc/articles/PMC3653006/


The arm URDF model is loaded and displayed with this launch command

```
roslaunch human_arm_description arm_display.launch
```

optional arguments:
- use_synergy: true | false (enable/disable single joint for finger grasp motion; defaults to true)
- scale: float (scale the model to an arbitrary size; defaults to 1.0)
- side: right | left (construct a left or right arm model)

The `motion_server` node can be used to save gestures with a service call or
used to replay gestures by linear interpolation either from the current pose or
an optionally provided starting gesture.

The `motion_director` node reads a config with a series of motions which get
dispatched to the `motion_server`.

author:
- David Leins dleins@techak.uni-bielefeld.de

original authors (human_hand):
- Guillaume Walck gwalck@techfak.uni-bielefeld.de
- Robert Haschke rhaschke@techfak.uni-bielefeld.de

original authors (pisa_hand):
- r.nuti@hotmail.it
- carlos@beta-robots.com
- matteo.bianchi@centropiaggio.unipi.it
- e.battaglia@centropiaggio.unipi.it
