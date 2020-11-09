#!/usr/bin/env python
import json


def make_sure_is_list(content):
    if type(content) is str:
        return [content]
    elif type(content) is list:
        if len(content) != 0:
            for element in content:
                if type(element) is not str:
                    raise TypeError("Content options must be strings.")
        return content
    else:
        raise TypeError("Content must be a string or list of strings.")


def get_database_from_file(file_path):
    try:
        with open(file_path, "r") as f:
            return json.load(f)
    except ValueError or IOError:
        return {}


def json_load_byteified(file_handle):
    return _byteify(
        json.load(file_handle, object_hook=_byteify),
        ignore_dicts=True
    )


def _byteify(data, ignore_dicts=False):
    if isinstance(data, unicode):
        return data.encode('utf-8')
    if isinstance(data, list):
        return [_byteify(item, ignore_dicts=True) for item in data]
    if isinstance(data, dict) and not ignore_dicts:
        return {
            _byteify(key, ignore_dicts=True): _byteify(value, ignore_dicts=True)
            for key, value in data.iteritems()
        }
    return data
