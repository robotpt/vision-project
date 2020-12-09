from abc import ABC, abstractmethod


class BaseMessenger(ABC):

    def __init__(self, name):
        self._name = name

    @property
    def name(self):
        return self._name

    @property
    @abstractmethod
    def is_active(self) -> bool:
        pass

    @abstractmethod
    def get_message(self):
        pass

    @abstractmethod
    def transition(self, arg) -> None:
        pass

    @abstractmethod
    def reset(self):
        pass