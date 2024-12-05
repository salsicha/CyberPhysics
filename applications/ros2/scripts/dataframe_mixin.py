from rosbags.serde.utils import Valtype
from typing import Callable, Union
AttrValue = Union[str, bool, int, float, object]


class DataframeMixIn():
    """
    Set of methods necessary to cast a ros topic into a pandas dataframe
    """
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

    @staticmethod
    def _create_plain_getter(key: str) -> Callable[[object], AttrValue]:
        """Create getter for plain attribute lookups.

        :param key: name of the field within a ros message
        """

        def getter(msg: object) -> AttrValue:
            return getattr(msg, key)

        return getter

    @staticmethod
    def _create_nested_getter(keys: list[str]) -> Callable[[object], AttrValue]:
        """Create getter for nested lookups.

        :param key: name of the field within a ros message
        """

        def getter(msg: object) -> AttrValue:
            value = msg
            for key in keys:
                value = getattr(value, key)
            return value

        return getter

    def _create_getters(self, keys: list[str], msgdef) -> list[Callable[[object], AttrValue]]:
        """
        Create a list of getters, one for each key in the argument keys.

        :param keys: field names to get from each ros message.
        :param msgdef: ros message definition, as returned by get_msgdef in rosbags.serde.messages
        :return: list of callable getters
        """

        getters = []
        for key in keys:
            subkeys = key.split('.')
            subdef = msgdef
            for subkey in subkeys[:-1]:
                subfield = next((x for x in subdef.fields if x.name == subkey), None)
                if not subfield:
                    raise Exception(f'Field {subkey!r} does not exist on {subdef.name!r}.')

                if subfield.descriptor.valtype != Valtype.MESSAGE:
                    raise Exception(f'Field {subkey!r} of {subdef.name!r} is not a message.')

                subdef = subfield.descriptor.args

            if subkeys[-1] not in {x.name for x in subdef.fields}:
                raise Exception(f'Field {subkeys[-1]!r} does not exist on {subdef.name!r}.')

            if len(subkeys) == 1:
                getters.append(self._create_plain_getter(subkeys[0]))
            else:
                getters.append(self._create_nested_getter(subkeys))
        return getters

    def dataframe_processor(self, msg, getters):
        """
        Format the message in order to be cast in a pandas dataframe.
        This method is explicitly used as message processor in Bagfile.topic_to_pandas.

        :param msg: ros message
        :param getters: function returned by the method _create_getters
        :return: message flatten in a list
        """
        return [x(msg) for x in getters]