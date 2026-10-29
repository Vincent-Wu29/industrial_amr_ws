#!/usr/bin/env python3
"""Create a complete Nav2 parameter file using the scenario runner's same overlay."""
import argparse
from pathlib import Path
import yaml


def overlay(base,changes):
    for key,value in changes.items():
        if isinstance(value,dict): overlay(base.setdefault(key,{}),value)
        else: base[key]=value


def main():
    root=Path(__file__).resolve().parents[1]
    parser=argparse.ArgumentParser()
    parser.add_argument('--profile',choices=['baseline','candidate'],required=True)
    parser.add_argument('--world',choices=sorted(p.stem for p in (root/'src/amr_simulation/worlds').glob('*.sdf')),default='warehouse')
    parser.add_argument('--mode',choices=['navigation','mission'],default='navigation')
    parser.add_argument('--output',required=True)
    args=parser.parse_args()
    if args.profile=='candidate' and (args.world!='narrow_aisle' or args.mode!='navigation'):
        parser.error('candidate parameters are scoped to narrow_aisle navigation; warehouse missions use baseline')
    config=root/'src/amr_navigation/config'
    params=yaml.safe_load((config/'nav2.yaml').read_text())
    if args.profile=='candidate': overlay(params,yaml.safe_load((config/'candidate.yaml').read_text()))
    destination=Path(args.output).expanduser().resolve(); destination.parent.mkdir(parents=True,exist_ok=True)
    destination.write_text(yaml.safe_dump(params),encoding='utf-8'); print(destination)


if __name__=='__main__': main()
