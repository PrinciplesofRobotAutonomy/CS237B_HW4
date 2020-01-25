import json
import os

def del_file(file_abs):
    """
    Helper function for deleting file.
    :param file_abs: str, the absolute file path
    :return: bool, true if it worked, false otherwise
    """
    try:
        os.remove(file_abs)
        print('Deleted file:', file_abs)
        return True
    except OSError:
        print('No such file:', file_abs)
        return False


def dump_json(json_file, dump, sort_keys=True, indent=4):
    """
    Write serialized scenario or warnings to file.
    :param json_file: str, the file name for the dump
    :param dump: the object which should be dumped to json
    :param sort_keys: bool, the sort key argument of json.dump
    :param indent: int, the indent argument of json.dump
    """
    print('Writing dump: %s' % json_file)
    with open(json_file, 'w') as handle:
        json.dump(dump, handle,  sort_keys=sort_keys, indent=indent)


def load_json(json_file):
    """
    Load json files.
    :param json_file: str, the file name to read
    """
    print('Reading load: %s' % json_file)
    with open(json_file, 'r') as handle:
        data = json.load(handle)
    return data

def maybe_makedirs(path_to_create):
    """This function will create a directory, unless it exists already,
    at which point the function will return.
    The exception handling is necessary as it prevents a race condition
    from occurring.
    Inputs:
        path_to_create - A string path to a directory you'd like created.
    """
    try:
        os.makedirs(path_to_create)
    except OSError:
        if not os.path.isdir(path_to_create):
            raise
