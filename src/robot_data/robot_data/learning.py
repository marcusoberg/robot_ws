from pathlib import Path
import json
import math
import os
import numpy as np
from .dataset import validate_dataset

def train(dataset, output, *, steps=100000, device="cpu", smoke=False, validate_every=1000):
    if not 1 <= steps <= 100000 or validate_every < 1:
        raise ValueError("Training requires 1–100000 updates and a positive validation interval")
    spec, contract = validate_dataset(dataset)
    if (contract["purpose"] == "smoke") != smoke:
        raise ValueError("Smoke datasets require explicitly selecting smoke training")
    output = Path(output)
    output.mkdir(parents=True,exist_ok=False)
    os.environ.setdefault("HF_DATASETS_CACHE",str((output/".cache/datasets").resolve()))
    import torch
    from torch.utils.data import DataLoader
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    from lerobot.policies.act.configuration_act import ACTConfig
    from lerobot.policies.factory import make_policy, make_pre_post_processors
    from lerobot.configs.types import FeatureType, PolicyFeature
    if device not in ("cpu","mps","cuda") or device=="cuda" and not torch.cuda.is_available() or device=="mps" and not torch.backends.mps.is_available():
        raise ValueError(f"Requested training device is unavailable: {device}")
    torch.manual_seed(1000)
    torch.set_num_threads(4)
    config = ACTConfig(device=device, push_to_hub=False, pretrained_backbone_weights=None)
    if smoke:
        config.chunk_size = config.n_action_steps = 10
        config.dim_model,config.n_heads,config.dim_feedforward = 64,4,128
        config.n_encoder_layers = config.n_decoder_layers = config.n_vae_encoder_layers = 1
    config.input_features = {"observation.state":PolicyFeature(FeatureType.STATE,(len(spec.joints),)),
        **{f"observation.images.camera_{k}":PolicyFeature(FeatureType.VISUAL,(3,spec.image_shape[0],spec.image_shape[1])) for k in spec.cameras}}
    offsets = {"action":[i*spec.period_ns/1e9 for i in range(config.chunk_size)]}
    train_ds = LeRobotDataset(contract["repo_id"],root=dataset,episodes=contract["train"],delta_timestamps=offsets)
    val_ds = LeRobotDataset(contract["repo_id"],root=dataset,episodes=contract["validation"],delta_timestamps=offsets)
    # Derive normalization exclusively from the training partition.
    stats = {}
    import pyarrow.dataset as pads
    table = pads.dataset(str(Path(dataset)/"data"),format="parquet").to_table(
        columns=["observation.state","action"],filter=pads.field("episode_index").isin(contract["train"]))
    for key in ("observation.state","action"):
        values = np.asarray(table[key].to_pylist(),dtype=np.float32)
        stats[key] = {"mean":values.mean(0),"std":values.std(0),"min":values.min(0),"max":values.max(0)}
    for camera in spec.cameras:
        stats[f"observation.images.camera_{camera}"] = {"mean":np.array([.485,.456,.406]).reshape(3,1,1),"std":np.array([.229,.224,.225]).reshape(3,1,1)}
    model = make_policy(config,ds_meta=train_ds.meta).to(device)
    pre, post = make_pre_post_processors(config,dataset_stats=stats)
    optimizer = torch.optim.AdamW(model.get_optim_params(),lr=1e-5,weight_decay=1e-4)
    train_loader = DataLoader(train_ds,batch_size=8,shuffle=True,num_workers=0)
    val_loader = DataLoader(val_ds,batch_size=8,shuffle=False,num_workers=0)
    iterator = iter(train_loader)
    best = math.inf
    history = []
    (output/"action_spec.json").write_text(spec.to_json())
    (output/"split.json").write_text(json.dumps(contract,indent=2))
    for step in range(1,steps+1):
        try:
            batch = next(iterator)
        except StopIteration:
            iterator = iter(train_loader); batch = next(iterator)
        model.train()
        loss, _ = model(pre(batch))
        if not torch.isfinite(loss):
            raise RuntimeError("Training loss is nonfinite")
        optimizer.zero_grad(set_to_none=True)
        loss.backward()
        torch.nn.utils.clip_grad_norm_(model.parameters(),10.)
        optimizer.step()
        if step % validate_every == 0 or step == steps:
            model.eval()
            total, count = 0.,0
            with torch.inference_mode():
                for batch in val_loader:
                    batch_loss,_ = model(pre(batch))
                    n = len(batch["observation.state"])
                    total += float(batch_loss)*n; count += n
            validation = total/count
            entry = dict(step=step,train_loss=float(loss.detach()),validation_loss=validation)
            history.append(entry)
            print(json.dumps(entry),flush=True)
            (output/"metrics.json").write_text(json.dumps(history,indent=2))
            checkpoint = output/"checkpoints"/f"{step:06}"
            model.save_pretrained(checkpoint)
            pre.save_pretrained(checkpoint,config_filename="policy_preprocessor.json")
            post.save_pretrained(checkpoint,config_filename="policy_postprocessor.json")
            (checkpoint/"action_spec.json").write_text(spec.to_json())
            (checkpoint/"training_contract.json").write_text(json.dumps(dict(format="robot-training.v2",dataset=contract,step=step,validation_loss=validation)))
            if validation < best:
                best = validation
                (output/"best.json").write_text(json.dumps({"checkpoint":str(checkpoint.resolve()),**entry},indent=2))
    return json.loads((output/"best.json").read_text())
