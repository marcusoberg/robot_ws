# PAROL6 visual meshes

These are byte-for-byte copies of the meshes referenced by `model/parol6.urdf`,
from the workspace's `parol6/meshes` description package (Source Robotics PAROL6).
That package declares `BSD` in its `package.xml`; no separate license text was
present in that local package. No geometry was regenerated or rescaled.

They are bundled here so the native MuJoCo deployment can resolve its assets
without ROS, a package index or an adjacent checkout. The visual origins match
our URDF, including the L6 flange's -0.021 m Z offset and the fixed vertical
pneumatic gripper body and jaws. Like the URDF, these tool visuals depict the
modeled fixed jaw positions, not measured pneumatic jaw motion.

The meshes only provide appearance. The existing primitive collision envelopes,
inertias and actuator dynamics remain an approximation, not validated CAD contact
geometry. Collision envelopes are in hidden visualization group 3; visual meshes
are in group 2 and contribute neither contacts nor mass.

| File | SHA-256 |
|---|---|
| `L1.STL` | `b5e0c0eda89b8302154c56a4af6817d4fb182f1d4be5118639901edee6424ebe` |
| `L2.STL` | `d65f55f604ee324bc48930a28904ced34da516354ba1afe43076a460ced14da6` |
| `L3.STL` | `f0ae4d6ceeb887899bc80de7b4ae9e89848efb182a0cab13512c907b0d12b4cc` |
| `L4.STL` | `654569735524e15752856254f60b01d961e8d66f3e72256e288edb741f64880f` |
| `L5.STL` | `a4b70a0cd1d32a74fd53a61dd7d46e823b4c3531f7abe881fa31c12a791c9c40` |
| `L6.STL` | `cc2af2d6c20207af3ea24ed554138a3c7ab8ef3b1b3f070243a98614b517c44f` |
| `base_link.STL` | `a7c6f497fc5a71e1b25539ca55ef0d5cce58171c8cff68af9744a4959bdb77bd` |
| `pneumatic_gripper_vertical_body_simplified.stl` | `69a18438e8cdd4cabda95d945cf8bbb995fd8663c61625b2146e4ff2818f4a09` |
| `pneumatic_gripper_vertical_left_jaw_simplified.stl` | `e354456f2213abc864b1316151c4739f3fbcba1ab39ecf9f59c06a9f77d37542` |
| `pneumatic_gripper_vertical_right_jaw_simplified.stl` | `ca21bc64e12ddb5c2a01ff661cdf19cc8abc608ebedd895e2683ec10f2550608` |

## URDF collision resources

The seven simplified arm meshes below are also copied byte-for-byte from the same
PAROL6 description package. The existing URDF references them for collision
geometry; including them makes those resource paths self-contained. This does not
change the URDF or MuJoCo scene configuration. The MuJoCo scene continues to use
its existing primitive collision envelopes.

| File | SHA-256 |
| --- | --- |
| `base_link_simplified.stl` | `8d614f615344762d28126b5f2c6a75032fe8b3ac35d663edf52c55e096a273f0` |
| `L1_simplified.stl` | `9afb21fda9db1f1b4a17d27b04a3797b59464eabf4c2d789af67824f78315942` |
| `L2_simplified.stl` | `edca0f662f1900e63d9d7b62f488533f20ac084716b7949fb44abc570e9905b7` |
| `L3_simplified.stl` | `0571dca4cc4e1f9f20dd28d08d8cfafe22cd6c97447fab7cb7aa0dab66d7cb22` |
| `L4_simplified.stl` | `bf07dc03f883a8251a3a9e545763305971c0779b30cb86d4de629770d0654925` |
| `L5_simplified.stl` | `4fee9e212b689a226739216e57ffe1faf40ba16303406633303c9131487dedbb` |
| `L6_simplified.stl` | `341fcb573dbe567209f69673bd301da03d4b7e9935b41a684d29a6cb5357d04a` |
