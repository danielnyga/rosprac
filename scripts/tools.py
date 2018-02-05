class RStorage(dict, object):
    '''
    Recursive extension of web.util.Storage that applies the Storage constructor
    recursively to all value elements that are dicts.
    '''
    __slots__ = ['_utf8']

    def __init__(self, d=None, utf8=False):
        self._utf8 = utf8
        if d is not None:
            for k, v in d.iteritems():
                self[k] = v

    def __setattr__(self, key, value):
        if key in type(self).__slots__:
            dict.__setattr__(self, key, value)
        else:
            self[key] = value

    def __setitem__(self, key, value):
        if self._utf8 and isinstance(key, str):
            key = key.encode('utf8')
        dict.__setitem__(self, key, rstorify(value, utf8=self._utf8))

    def __getattr__(self, key):
        if key in type(self).__slots__:
            return dict.__getattr__(self, key)
        else:
            try:
                return self[key]
            except KeyError:
                raise AttributeError('%s object has no attribute %s' % (type(self).__name__, key))

    def __delattr__(self, key):
        try:
            del self[key]
        except KeyError as k:
            raise (AttributeError, key)

    def __repr__(self):
        return ('<%s ' % type(self).__name__) + dict.__repr__(self) + '>'


def rstorify(e, utf8=True):
    if type(e) is dict:
        return RStorage(d=e, utf8=utf8)
    elif type(e) in (list, tuple):
        return [rstorify(i, utf8=utf8) for i in e]
    else:
        return e