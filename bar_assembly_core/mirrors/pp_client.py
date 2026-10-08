"""
Pointing pybullet_planning at one PyBullet world for a block of code.

pybullet_planning keeps one client for the whole process, and most of its modules copy it when imported
(`from pybullet_planning.utils import CLIENT`), so `pp.set_client` alone leaves them on the old world.
`pp_client` sets it in every loaded pybullet_planning module and restores each one afterwards.

! One `pp` user at a time in the whole process: the client is global.
"""

from __future__ import annotations

import sys
from contextlib import contextmanager
from typing import Iterator

import pybullet_planning as pp


@contextmanager
def pp_client(client_id: int) -> Iterator[None]:
    """Inside the block, every pybullet_planning function acts on the world `client_id`; the previous one after.

    Args:
        client_id: A PyBullet client id.

    Yields:
        None
    """
    modules = [module for name, module in list(sys.modules.items())
               if name.split(".")[0] == "pybullet_planning" and hasattr(module, "CLIENT")]
    previous = [(module, module.CLIENT) for module in modules]
    pp.set_client(client_id)
    for module in modules:
        module.CLIENT = client_id
    try:
        yield
    finally:
        for module, client in previous:
            module.CLIENT = client
