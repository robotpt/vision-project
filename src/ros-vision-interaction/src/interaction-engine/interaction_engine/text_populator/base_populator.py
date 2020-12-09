from abc import ABC, abstractmethod
from robotpt_common_utils import lists


class BasePopulator(ABC):

    def __init__(self, main_tags, option_tags):
        main_tags = lists.make_sure_is_iterable(main_tags)
        option_tags = lists.make_sure_is_iterable(option_tags)

        for tag in main_tags:
            if tag in option_tags:
                raise ValueError("Option tags cannot be in main tags")

        for tag in option_tags:
            if tag in main_tags:
                raise ValueError("Main tags cannot be in option tags")

        self._main_tags = main_tags
        self._option_tags = option_tags

    @abstractmethod
    def get_replacement(self, tags_dict) -> str:
        pass

    def is_tags_valid(self, tags_dict) -> bool:
        # TODO: make tags more sophisticated so that they can be tested individually

        if type(tags_dict) is not dict:
            raise TypeError

        num_in_main = 0
        for tag in tags_dict.keys():
            if tag in self._main_tags:
                num_in_main += 1
            elif tag in self._option_tags:
                pass
            else:
                return False

        return num_in_main is 1
