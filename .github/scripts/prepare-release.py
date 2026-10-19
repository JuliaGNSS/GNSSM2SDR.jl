#!/usr/bin/env python3
"""Write a release computed by a semantic-release dry run into the repository.

Sets the `version` of both packages, GNSSM2SDR (Project.toml) and M2SDRLoop
(M2SDRLoop/Project.toml), which are always released together at the same
version, makes GNSSM2SDR require the M2SDRLoop of that version, and puts the
release notes at the top of CHANGELOG.md, as @semantic-release/changelog and
the shared configuration's Project.toml replacement would. Unlike them, it does not tag: TagBot tags the
release once the General registry has merged it.

Until then the version is pending. A later run computes the next version from
the same last tag, over all commits since, so its notes cover the pending
release too. Every CHANGELOG.md section newer than the last tag is therefore
replaced rather than kept.

Usage: prepare-release.py <next version> <last released version or "">
The release notes are read from the RELEASE_NOTES environment variable.
Prints "changed" if a file was modified and "unchanged" otherwise.
"""
import os
import re
import sys

PROJECTS = ("Project.toml", "M2SDRLoop/Project.toml")
TITLE = "# Changelog"
# Release headings: "# 1.0.0 (date)" for minor and major releases,
# "## [1.0.1](compare-url) (date)" for patch releases.
HEADING = re.compile(r"^#{1,2} \[?(\d+)\.(\d+)\.(\d+)", re.MULTILINE)
DATE = re.compile(r"\(\d{4}-\d{2}-\d{2}\)")


def parse(version):
    return tuple(int(part) for part in version.split("."))


def set_m2sdrloop_compat(content, version):
    """GNSSM2SDR requires the M2SDRLoop released with it."""
    section = re.search(r"^\[compat\]\n(.*?)(?=^\[|\Z)", content, re.MULTILINE | re.DOTALL)
    if section is None:
        sys.exit("Project.toml has no [compat] section")
    body, count = re.subn(
        r'^M2SDRLoop\s*=\s*"[^"]*"',
        f'M2SDRLoop = "{version}"',
        section.group(1),
        flags=re.MULTILINE,
    )
    if count != 1:
        sys.exit(f"Project.toml must have exactly one M2SDRLoop compat entry, found {count}")
    return content[: section.start(1)] + body + content[section.end(1) :]


def set_project_version(path, version):
    with open(path) as f:
        content = f.read()
    new, count = re.subn(
        r'^version\s*=\s*"\d+(\.\d+){2}"',
        f'version = "{version}"',
        content,
        flags=re.MULTILINE,
    )
    if count != 1:
        sys.exit(f"{path} must have exactly one version field, found {count}")
    if path == "Project.toml":
        new = set_m2sdrloop_compat(new, version)
    with open(path, "w") as f:
        f.write(new)
    return new != content


def update_changelog(notes, last_version):
    try:
        with open("CHANGELOG.md") as f:
            content = f.read()
    except FileNotFoundError:
        content = ""
    body = content.strip()
    if body.startswith(TITLE):
        body = body[len(TITLE) :].strip()
    # Drop the sections of versions that were never released.
    last = parse(last_version) if last_version else None
    for match in HEADING.finditer(body):
        if last is not None and tuple(map(int, match.groups())) <= last:
            body = body[match.start() :]
            break
    else:
        body = ""
    new = f"{TITLE}\n\n{notes.strip()}\n" + (f"\n{body.strip()}\n" if body else "")
    # The notes carry the date of the run. A pending release whose notes
    # differ only in that date has not changed and is not registered again.
    if DATE.sub("", new) == DATE.sub("", content):
        return False
    with open("CHANGELOG.md", "w") as f:
        f.write(new)
    return True


def main():
    version, last_version = sys.argv[1], sys.argv[2]
    notes = os.environ["RELEASE_NOTES"]
    changed = False
    for path in PROJECTS:
        changed = set_project_version(path, version) or changed
    changed = update_changelog(notes, last_version) or changed
    print("changed" if changed else "unchanged")


if __name__ == "__main__":
    main()
