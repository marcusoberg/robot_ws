# Bringup launch composition

`system.launch.py` includes the launch files owned by Robot Server, recorder, examples and Model Client. It does not configure runtime internals or manage episodes. Optional `record`, `example` and `model` flags only start those independent processes.

Presets `so101_act.launch.py`, `so101_smolvla.launch.py` and `parol6.launch.py` choose robot camera deployment and model input configuration. `mode:=stepped` makes advancement explicit. Launch does not enable the robot or select a producer.

Start Model Server in its isolated environment separately; pass `model_url` to bringup or Model Client. Keyboard teleop needs its own interactive terminal.
