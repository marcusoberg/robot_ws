import argparse
import json
from pathlib import Path

from robot_core.data.bundle import export, verify
from robot_core.data.inference import Features
from .lerobot import LeRobotModel


def load_local(bundle, device):
    root=Path(bundle)
    manifest=verify(root)
    features=Features.from_json((root/"features.json").read_text())
    model=LeRobotModel.load(str(root/"model"),features=features,revision=manifest["revision"],device=device,model_type="act")
    model.revision=manifest["revision"]
    return model

def main():
    p=argparse.ArgumentParser(description="Export an ACT checkpoint with its v2 action contract")
    p.add_argument("--checkpoint",required=True);p.add_argument("--output",required=True)
    print(json.dumps(export(**vars(p.parse_args())),indent=2))
if __name__=="__main__":main()
