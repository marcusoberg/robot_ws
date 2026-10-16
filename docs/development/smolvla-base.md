# Original SmolVLA base checkpoint integration

Date: 2026-10-15. This setup is incomplete for robot control: upstream statistics
and their embodiment calibration are missing/ambiguous. Do not replace this gap
with arbitrary statistics from ACT, our fine-tunes, or simulator joint limits.

## Immutable source

- Repository: https://huggingface.co/lerobot/smolvla_base
- Download revision: `5e8d12a6e2975b0e5e5fce7c8caf47c371d257b6`
- Weights SHA-256: `7cd549ac2351fb069c0ddb3c34ad2d09cfc92b56a15dccdfc2e41467aaca01eb`
- Config SHA-256: `650584b56c104720f7a3c91d1ec6bec9e8de8ac11e60c92ba2fa82d93eda147d`
- Preprocessor JSON SHA-256: `7683d648280a72f0d51e869fb8dd1596beed90b7f84849330cf1bc26634a4bd6`
- Postprocessor JSON SHA-256: `2b78bb742065288df2ec63b0ee35f97a1f6950171cf2272f7e34b1fe3873b17b`
- Both processor Safetensors SHA-256: `490ab239d96e263687c0b2e386a0afbc235a2eceb9857c36ed32f2f162a3e7c8`

The new configuration references the downloaded files directly; no checkpoint
file is rewritten. Only the in-memory device and `load_vlm_weights=False` override
are applied, as in the existing serving adapter. The full policy already contains
the VLM weights.

## Missing semantic mapping

Both processor state files contain mean/std entries only for:

- `so100.buffer.action`
- `so100-blue.buffer.action`
- `so100-red.buffer.action`

Neither contains `observation.state.mean/std` or `action.mean/std`. In installed
LeRobot 0.6.0, `_NormalizationMixin._apply_transform` returns its input unchanged
when the feature key is absent from `_tensor_stats`. The checkpoint declares
MEAN_STD for state and actions, so this is not an intentional IDENTITY mapping.
Upstream reports the same issue:
https://github.com/huggingface/lerobot/issues/4415 and
https://github.com/huggingface/lerobot/issues/2374.

The diagnostic deliberately supplies captured runtime positions unchanged and
returns the published processor output without claiming physical units. This
does not establish a valid normalized state distribution. No degree/radian,
motor-range or gripper conversion has been invented. Robot control still needs
the intended state statistics, an explicit action-statistics selection, and the
corresponding calibrated joint zero/sign/scale/gripper convention. Alternatively,
a separately justified target-dataset preprocessing experiment would be a new,
explicit deployment choice, not a reproduction of the release's preprocessing.

## Input preparation and output isolation

The paper's camera priority is top, wrist, then additional/side views:
https://huggingface.co/blog/smolvla. We map the available front view to the third
slot, preserving calibration provenance. Bilinear letterboxing preserves aspect
ratio; the source images are not changed. Original calibration describes the
original pixels, not the prepared tensors.

`policy.v1` features/predictions now additionally support the explicitly named
`unmapped_model_output` representation for diagnostics. The decoder requires the
prediction representation to match negotiation. Existing joint_position profiles
retain their default. The ROS Model Client rejects unmapped output profiles
before producer registration. The Model Server refuses a joint_position contract
if declared processor normalization statistics cannot bind to state/actions.
Older peers which only accept joint_position reject diagnostic negotiation.

The diagnostic period is 1/30 s, following the pretraining data rate described
upstream. No scheduling or robot timing is performed with it.

## Verification boundary

The implementer checks syntax/configuration, processor coverage, installed import
provenance and an offline CPU load of the original checkpoint with strict weight
loading. The user performs MPS predictions, ROS captures and all operational
checks. No unit tests, training, runtime/URDF modifications or robot commands are
part of this change. This does not qualify task success or physical deployment.

Completed static checks: both changed ROS packages built successfully; installed
launch/profile files match source; Python/JSON/YAML parsing and diff whitespace
checks pass. Input preparation produces three 256×256 RGB copies while preserving
read-only 640×480 source arrays. Original ACT and the historical SmolVLA fine-tune
both retain valid state/action processor bindings; the base reports both missing.
Strict offline CPU checkpoint loading succeeds. Model weights match the Hub
download SHA-256 above. No inference or robot operation was run by the implementer.
