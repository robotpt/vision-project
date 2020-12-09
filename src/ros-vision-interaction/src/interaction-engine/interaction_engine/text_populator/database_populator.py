from interaction_engine.json_database import Database
from interaction_engine.text_populator.base_populator import BasePopulator

import datetime


class DatabasePopulator(BasePopulator):

    class Tags:
        MAIN = 'db'
        POST_OP = 'post-op'
        DEFAULT_VALUE = 'default value'
        CONVERT_FN = 'convert fn'

    def __init__(
            self,
            database,
    ):

        super().__init__(
            main_tags=[self.Tags.MAIN],
            option_tags=[
                self.Tags.POST_OP,
                self.Tags.DEFAULT_VALUE,
            ]
        )

        if type(database) is str:
            self._db = Database(database)
        else:
            self._db = database

    _common_fns = {
        'increment': lambda x: x+1,
        'decrement': lambda x: x-1,
    }

    def get_replacement(
            self,
            key,
            default_value=None,
            modify_before_resaving_fn=None,
    ):
        if self._db.is_set(key):
            value = self._db[key]
        elif default_value is not None:
            value = default_value
        else:
            raise KeyError(f"No value for key '{key}' found and no default value provided")

        if modify_before_resaving_fn is not None:
            if modify_before_resaving_fn in DatabasePopulator._common_fns:
                modify_before_resaving_fn = DatabasePopulator._common_fns[modify_before_resaving_fn]
            new_value = modify_before_resaving_fn(value)
            self._db[key] = new_value

        if type(value) is datetime.time:
            value = value.strftime("%-I:%M%p")

        return str(value)

    def __contains__(self, key):
        return key in self._db
