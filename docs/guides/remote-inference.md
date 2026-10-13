# Remote inference

Use the [remote training and deployment walkthrough](remote-models.md) for SSH transfer, the isolated model environment, serving, local port forwarding, training/resume and retrieval.

The Model Server runs on remote loopback. Only `policy.v1` requests/predictions cross the SSH tunnel; the Robot Server, native runtime and ROS discovery stay local. Start with the [prepared local models](../tutorials/04-local-inference.md).
