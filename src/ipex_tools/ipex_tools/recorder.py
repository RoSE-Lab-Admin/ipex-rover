"""Bag recording with a standard topic set and a notes file.

Each recording is a folder:
    <folder>/bag/        ros2 bag (MCAP), open in Foxglove or with rosbags
    <folder>/notes.yaml  what, when, git commit, notes
"""

import datetime
import os
import signal
import subprocess
import time

import yaml


# Everything needed for time/energy characterization and debugging.
DEFAULT_TOPICS_REGEX = (
    r'/rover/.*|/ipex/.*|/joint_states|/cmd_vel.*|/joy|'
    r'/diff_drive_controller/odom|/diff_drive_controller/cmd_vel|'
    r'/(front|rear)_(drum|shoulder)_controller/commands|'
    r'/tf|/tf_static|/rosout'
)

DATA_DIR = os.path.expanduser(os.environ.get('ROXEY_DATA_DIR', '~/roxey_data'))


def timestamp():
    return datetime.datetime.now().strftime('%Y-%m-%d_%H%M%S')


def new_folder(kind, name, exact=False):
    """~/roxey_data/<kind>/<timestamp>_<name>/ (created).

    exact=True uses <name> as given, adding _2, _3, ... if it already exists.
    """
    safe = ''.join(c if c.isalnum() or c in '-_.' else '_' for c in name) or 'unnamed'
    base = os.path.join(DATA_DIR, kind, safe if exact else f'{timestamp()}_{safe}')
    folder, n = base, 1
    while exact and os.path.exists(folder):
        n += 1
        folder = f'{base}_{n}'
    os.makedirs(folder, exist_ok=True)
    return folder


def git_info():
    """Commit of the workspace that is running, best effort."""
    try:
        from ament_index_python.packages import get_package_prefix
        workspace = os.path.dirname(os.path.dirname(get_package_prefix('ipex_tools')))
        commit = subprocess.run(['git', '-C', workspace, 'rev-parse', '--short', 'HEAD'],
                                capture_output=True, text=True, timeout=3).stdout.strip()
        dirty = subprocess.run(['git', '-C', workspace, 'status', '--porcelain'],
                               capture_output=True, text=True, timeout=3).stdout.strip()
        return {'commit': commit or 'unknown', 'uncommitted_changes': bool(dirty)}
    except Exception:   # noqa: BLE001 - recording must never fail on this
        return {'commit': 'unknown', 'uncommitted_changes': None}


class BagRecorder:

    def __init__(self, folder, notes='', meta=None, topics_regex=DEFAULT_TOPICS_REGEX,
                 all_topics=False):
        self.folder = folder
        self.bag_dir = os.path.join(folder, 'bag')
        self.notes = notes
        self.meta = meta or {}
        self.topics_regex = topics_regex
        self.all_topics = all_topics    # True: every topic (ros2 bag record -a)
        self.proc = None
        self.started = None

    def start(self, warmup_s=1.5):
        log = open(os.path.join(self.folder, 'record.log'), 'w')
        selection = ['-a'] if self.all_topics else ['-e', self.topics_regex]
        self.proc = subprocess.Popen(
            ['ros2', 'bag', 'record', '-o', self.bag_dir,
             *selection, '--include-hidden-topics'],
            stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        self.started = datetime.datetime.now()
        self._write_notes()
        time.sleep(warmup_s)        # let the recorder discover and subscribe
        if self.proc.poll() is not None:
            raise RuntimeError(f'ros2 bag record exited; see {log.name}')
        return self

    def stop(self, extra=None):
        if self.proc is not None and self.proc.poll() is None:
            os.killpg(self.proc.pid, signal.SIGINT)
            try:
                self.proc.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(self.proc.pid, signal.SIGKILL)
        if extra:
            self.meta.update(extra)
        self._write_notes(stopped=datetime.datetime.now())

    def _write_notes(self, stopped=None):
        notes = {
            'started': self.started.isoformat(timespec='seconds') if self.started else None,
            'stopped': stopped.isoformat(timespec='seconds') if stopped else None,
            'notes': self.notes,
            'topics': 'all' if self.all_topics else self.topics_regex,
            **git_info(),
            **self.meta,
        }
        with open(os.path.join(self.folder, 'notes.yaml'), 'w') as f:
            yaml.safe_dump(notes, f, sort_keys=False)

    def __enter__(self):
        return self.start()

    def __exit__(self, *_exc):
        self.stop()
