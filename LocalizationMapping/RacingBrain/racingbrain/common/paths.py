import os
from pathlib import Path


WORKSPACE_MARKERS = (
    "LocalizationMapping/slam",
    "LocalizationMapping/perception",
    "LocalizationMapping/RacingBrain",
)


def _candidate_roots(start):
    if start:
        yield Path(start).expanduser().resolve()

    for env_name in ("RACINGBRAIN_WORKSPACE", "DRD26_WORKSPACE"):
        value = os.environ.get(env_name)
        if value:
            yield Path(value).expanduser().resolve()

    yield Path.cwd().resolve()
    yield Path(__file__).resolve()


def find_workspace_root(start=None):
    seen = set()
    for candidate in _candidate_roots(start):
        for path in (candidate, *candidate.parents):
            if path in seen:
                continue
            seen.add(path)
            if all((path / marker).exists() for marker in WORKSPACE_MARKERS):
                return path
    return None


def workspace_path(relative_path, start=None):
    root = find_workspace_root(start)
    if root is None:
        return Path(relative_path)
    return root / relative_path
