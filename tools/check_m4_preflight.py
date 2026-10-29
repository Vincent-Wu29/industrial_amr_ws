#!/usr/bin/env python3
"""Require a separate functional pilot to support the registered route hypothesis."""
import argparse
import json
from pathlib import Path


def main():
    parser=argparse.ArgumentParser(); parser.add_argument('--input',required=True)
    args=parser.parse_args(); samples={}
    for path in Path(args.input).rglob('metrics.json'):
        sample=json.loads(path.read_text()); profile=sample['profile']
        if profile in samples: raise ValueError('duplicate pilot profile; retain it in a separate cohort')
        samples[profile]=sample
    if set(samples)!={'baseline','candidate'}: raise ValueError('both pilot profiles required')
    for sample in samples.values():
        if not (sample['passed'] and sample['collision_count']==0 and sample['truth_valid'] and sample['physical_stop_verified']):
            raise ValueError('functional pilot assertions failed; formal experiment has not started')
    if samples['candidate'].get('route')!='corridor':
        raise ValueError('pilot does not support the registered corridor hypothesis; formal experiment has not started')
    print('Both pilots passed; candidate corridor traversal independently observed. No performance claim from pilots.')


if __name__=='__main__': main()
