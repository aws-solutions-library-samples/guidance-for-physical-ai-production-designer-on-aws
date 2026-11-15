"""Bounded in-memory copies of checksummed, unchanged component files."""
from collections import OrderedDict
from copy import deepcopy
import threading

from .store import Store, digest


class RuntimeStore(Store):
    """Reuse validated JSON decoding, never complete requests or mutable objects.

    Every miss or file replacement passes through Store's schema/dependency/
    checksum validation. A cached value is an isolated copy of that validated
    payload; callers cannot mutate future reads. Removing a file invalidates it.
    """
    def __init__(self, root, max_bytes=2_000_000_000, memory_bytes=64*1024*1024):
        super().__init__(root, max_bytes=max_bytes)
        self.memory_bytes = memory_bytes
        self._memory = OrderedDict()
        self._memory_size = 0
        self._memory_lock = threading.RLock()
        self.memory_hits = 0

    def get(self, layer, dependencies):
        key = digest(dependencies)
        address = layer, key
        path = self.path(layer, key)
        try:
            stat = path.stat()
            signature = stat.st_dev, stat.st_ino, stat.st_size, stat.st_mtime_ns, stat.st_ctime_ns
        except OSError:
            signature = None
        with self._memory_lock:
            previous = self._memory.pop(address, None)
            if previous is not None:
                if signature == previous[0]:
                    self._memory[address] = previous
                    self.hits += 1
                    self.memory_hits += 1
                    return deepcopy(previous[2])
                self._memory_size -= previous[1]
        payload = super().get(layer, dependencies)
        if payload is None or signature is None:
            return payload
        size = signature[2]
        if size <= self.memory_bytes:
            with self._memory_lock:
                previous = self._memory.pop(address, None)
                if previous is not None:
                    self._memory_size -= previous[1]
                while self._memory and self._memory_size+size > self.memory_bytes:
                    _, (_, removed_size, _) = self._memory.popitem(last=False)
                    self._memory_size -= removed_size
                self._memory[address] = signature, size, deepcopy(payload)
                self._memory_size += size
        return payload
