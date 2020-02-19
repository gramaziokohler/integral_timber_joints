def _serialize_to_data(obj):
    return dict(
        dtype='{}/{}'.format(obj.__class__.__module__, obj.__class__.__name__),
        data=obj.to_data()
    )


def _deserialize_from_data(data):
    module, attr = data['dtype'].split('/')
    cls = globals().get(attr)

    if cls is None:
        cls = getattr(__import__(module, fromlist=[attr]), attr)

    return cls.from_data(data['data'])
