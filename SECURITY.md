# Security policy

## Supported versions

Security fixes go into the latest release and `main`. Compile Driver always builds the current `main`.

## What counts

PSVR2-DriverKit runs with your user's rights. A few steps ask for administrator rights, and each one asks first:

- **Install dependencies:** runs the distribution's package manager and installs udev rules, through `pkexec` or
  `sudo`.
- **Allow high priority:** runs `setcap CAP_SYS_NICE` on SteamVR's `vrcompositor-launcher`.

Please report, for example:

- a way for another local user, or a file the user downloads, to make these privileged steps do more than described;
- udev rules that give device access wider than intended;
- code that downloads and runs something without verifying it.

The hand-tracking models are downloaded with SHA-256 checks.

## How to report

Please don't open a public issue for a vulnerability. Use GitHub's private reporting instead: **Security** tab →
**Report a vulnerability** (<https://github.com/mkooner/PSVR2-DriverKit/security/advisories/new>). You'll get an
answer as soon as possible. Once a fix is out, the advisory can be published with credit to you if you wish.
