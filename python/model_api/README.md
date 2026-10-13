# Model API

This Python library defines `policy.v1` requests and predictions, validates camera/joint features, decodes Robot Server captures, and connects to a model server over WebSockets. It has no ROS, SDK, recorder, LeRobot or Torch dependency.

`RemoteModel` is the synchronous client. `await AsyncRemoteModel.connect(url, features)` creates the asynchronous client; `await client.predict(input)` returns a prediction and measured round-trip/computation timing. Every prediction retains the capture identity, execution time, action period and checkpoint hash negotiated by the connection.

`decode_capture_packet(packet)` reads the episode-free `robot.capture.v2` packet. `policy_input(capture, features, cameras, instruction, episode, context)` maps its named RGB cameras and joint measurement into the selected model's input. Depth can remain in the capture while RGB-only models select their input planes.

Install through the shared workspace setup. ROS callers live in `src/model_client`; model implementations live in `python/model_server`.
