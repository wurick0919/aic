# AIC with RGBD

This project is an application utilizing  [**AI for Industry Challenge toolkit**](https://github.com/intrinsic-dev/aic). **AIC** is an open competition held by Intrinsic and Open Robotics, for developers and roboticists aimed at solving some of the hardest, high-impact problems in robotics and manufacturing.

The aim of the project is to conduct (some cool name like vision based insersion task?). We utilise SAM2 and FoundationPose to achive XXX?
(an demonstration video?)


## Architechture

We modify gazebo to expose depth. Instead of using 3 cameras, we only use one rgbd. Use sam2 to create high quality mask(an image of the mask), and Foundation pose to predict target pose, using the target pose and PI control to guide the robot to finish insertion task. (maybe an )


## Environment and setup
Using aws ec2, g5.2xlarge. mention the gpu, ram and vram.

This project shares a similar environment setup with AIC toolkit, uses pixi env to manage dependancies, and distrobox for simulation env, where gazebo is run.
Some part of the dependencies of SAM2 and FP are also added, but other package needed to be manually installed.

## Technical stacks?
ros2 kilted kaiju
pixi
distrobox
gazebo
OpenCV
SAM
FP

## Challanges
talk about grey frame issue

## Repo Structure

Most of the work is being done in my_policy_node pkg, where aic_model_depth is the main node, and MyFoundationPosePolicy contian the actual workflow.



## License

This project is licensed under the Apache License 2.0 - see the individual package files for details.