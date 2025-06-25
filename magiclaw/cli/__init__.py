"""
MagiClaw CLI.
=========

MagiClaw CLI is a command line interface for running the MagiClaw system.
It provides a simple way to start the MagiClaw with various configurations.

Usage
-----------------

To run MagiClaw, use the following command:

```
run-magiclaw --id <id> --mode <mode> --loop_rate <loop_rate> --phone_host <phone_host> --bilateral_host <bilateral_host>
```

where `<id>` is the MagiClaw's ID (corresponding to the `CAN` interface, default is 0), `<mode>` is the operation mode
(including `standalone` and `teleop`, default is `standalone`), `<loop_rate>` is the loop rate in Hz (default is 30),
`<phone_host>` is the host address for the phone (overwrite the default value in the config file, optional), and
`<bilateral_host>` is the host address for the bilateral communication (only required in `teleop` mode, overwrite the
default value in the config file, optional).

Note that if the CLI is installed under `venv` or `conda`, make sure to setup the environment first, or directly run
the script with the environment directory, e.g., for a `miniconda` environment:

```
/home/user/miniconda3/bin/run-magiclaw
```
"""

from .run import main as run_magiclaw   # noqa