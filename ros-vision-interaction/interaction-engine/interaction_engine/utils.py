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
