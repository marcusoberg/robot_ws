# SO101 collision meshes

The calibrated MuJoCo robot uses 24 convex collision hulls under
`assets/collision/`: 12 for `wrist_roll_follower_so101_v1` and 12 for
`moving_jaw_so101_v1`. They preserve the upstream CoACD decomposition.
`so101_new_calib.xml` declares and uses every hull. The 13 top-level STL files
provide the robot's visual meshes and the remaining link collision geometry.

The XML separates visual geometry in group 2 from collision geometry in group
3. The meshes are shared by both deployed scenes. No mesh generation program
or simulator wrapper is needed to use them.

The original asset source and revision are recorded in [PROVENANCE.md](PROVENANCE.md).
Attribution and license notices accompany these files. This note describes the
retained geometry; it makes no new simulation or physical qualification claim.
