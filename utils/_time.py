from dataclasses import dataclass
import time


@dataclass
class _Time:
    delta_time: float = 0
    prev_time: float = time.time()

    def update(self) -> None:
        self.delta_time = time.time() - self.prev_time
        self.prev_time = time.time()

    @property
    def fps(self) -> float:
        if self.delta_time:
            return 1/self.delta_time
        return 0


Time = _Time()
