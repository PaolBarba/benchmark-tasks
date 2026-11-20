#!/usr/bin/env python3
"""Run Dream-1 on the native pencil-case task and export the overhead stream."""

import argparse
import hashlib
from importlib.metadata import version
import json
import math
from pathlib import Path
import socket
import subprocess
import sys
import time

from inspect_robots._video import discover_streams, encode_stream, resolve_frames_dir
from resist_isaac.runtime import _task_module

SEED = 0
MODEL = 'dreamlabs/dream-1-beta'


def output_text(*command):
    return subprocess.check_output(command, text=True).strip()


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def run(task, output):
    task = task.resolve()
    output = output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    cfg = json.loads((task / 'pilot.json').read_text())
    _, fingerprint = _task_module(task)
    config = output / 'config.ini'
    config.touch()
    cli = str(Path(sys.executable).with_name('inspect-robots'))
    evaluation = output / 'evaluation'
    command = [cli, 'run', '--config', str(config), '--embodiment', 'resist_isaac', '--policy', 'dream',
               '-P', f'model={MODEL}', '-P', 'dry_run=false', '-P', 'record=false',
               '-E', f'task={task}', '-E', 'object_jitter=0', '-E', f'control_hz={cfg["control_hz"]}',
               '-E', f'device={cfg.get("device", "cuda:0")}',
               '-E', f'cam_width={cfg.get("cam_width", 640)}', '-E', f'cam_height={cfg.get("cam_height", 360)}',
               '--instruction', cfg['instruction'],
               '--scorer', 'task_score', '--seed', str(SEED), '--max-steps', str(cfg['max_steps']),
               '--store-frames', '--no-prompt', '--grader', 'none', '--no-rerun', '--no-rerun-save',
               '--fail-on-error', '1', '--log-dir', str(evaluation)]
    receipt = dict(status='starting', hostname=socket.gethostname(), policy='dream',
                   task_fingerprint=fingerprint, seed=SEED, max_steps=cfg['max_steps'],
                   git_revision=output_text('git', 'rev-parse', 'HEAD'),
                   gpu=output_text('nvidia-smi', '--query-gpu=index,name,driver_version', '--format=csv,noheader'),
                   versions={pkg: version(pkg) for pkg in ('isaacsim', 'isaaclab', 'isaaclab-arena', 'yam-arena',
                                                            'inspect-robots', 'inspect-robots-dream', 'dreamlabs-client')},
                   command=command)
    receipt_path = output / 'receipt.json'
    receipt_path.write_text(json.dumps(receipt, indent=2) + '\n')
    started = time.monotonic()
    try:
        with (output / 'run.log').open('w') as log:
            subprocess.run(command, stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT, check=True)
        receipt.update(verify(evaluation, output, fingerprint, cfg))
        receipt['status'] = 'completed'
    except Exception as error:
        receipt.update(status='error', error=str(error))
        raise
    finally:
        receipt['elapsed_wall_s'] = time.monotonic() - started
        receipt['source_unchanged'] = _task_module(task)[1] == fingerprint
        receipt_path.write_text(json.dumps(receipt, indent=2) + '\n')
    if not receipt['source_unchanged']:
        raise RuntimeError('Task source changed during the Dream-1 trial')


def verify(evaluation, output, fingerprint, cfg):
    log_path, = [p for p in evaluation.glob('*.json') if '.live.' not in p.name]
    data = json.loads(log_path.read_text())
    sample, = data['samples']
    spec = data['eval']
    result = data['results']
    if (data['status'] != 'success' or sample['status'] != 'success' or result['errored_trials'] != 0
            or result['total_trials'] != 1 or spec['environment_revision'] != fingerprint[:12]
            or spec['seed'] != SEED or spec['max_steps'] != cfg['max_steps']):
        raise RuntimeError('The evaluation log is incomplete or its run settings do not match')
    transcript, = sample['policy_transcripts']
    if transcript['model'] != MODEL or transcript['dry_run'] is not False or not transcript['requests']:
        raise RuntimeError('Evaluation recorded no authenticated Dream-1 requests')
    score = result['metrics']['task_score']
    steps = data['stats']['total_steps']
    if not (score in (0, .5, 1) and score == sample['epochs'][0]['task_score']
            and 0 < steps <= cfg['max_steps']):
        raise RuntimeError('Recorded score or step count falls outside the task rubric')

    trajectory = evaluation / sample['trial_metadata'][0]['actions']
    header, *controls = [json.loads(line) for line in trajectory.read_text().splitlines()]
    if (header['action_dim'] != 14 or len(controls) != steps
            or any(len(row['action']) != 14 or not all(map(math.isfinite, row['action'])) for row in controls)):
        raise RuntimeError('Missing or invalid executed Dream-1 joint targets')

    frames = resolve_frames_dir(data['stats']['frames_dir'], log_path)
    if frames is None:
        raise RuntimeError('Dream-1 camera frames are unavailable')
    streams, _ = discover_streams(frames)
    top, = [stream for name, stream in streams.items() if name.endswith('_top_cam')]
    if [step for step, _ in top] != list(range(steps + 1)):
        raise RuntimeError('The overhead stream has missing or repeated frame indices')
    video = output / 'top_cam.mp4'
    encoded = encode_stream(top, video, cfg['control_hz'], 'ffmpeg')
    if encoded.error or encoded.piped != len(top) or video.stat().st_size >= 50_000_000:
        raise RuntimeError(f'Overhead video failed export: {encoded}')

    return dict(score=score, steps=steps, simulated_s=steps / cfg['control_hz'],
                evaluation_wall_s=data['stats']['duration_s'], termination=sample['termination_reasons'][0],
                requests=len(transcript['requests']), model=MODEL, video_frames=encoded.piped,
                video_sha256=sha256(video), eval_log=str(log_path.relative_to(output)),
                eval_log_sha256=sha256(log_path), trajectory=str(trajectory.relative_to(output)),
                trajectory_sha256=sha256(trajectory))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('task', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    run(args.task, args.output)
