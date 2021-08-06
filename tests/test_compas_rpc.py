import compas
from compas.geometry import add_vectors
result = add_vectors([2, -1, 3], [1, 0, 1])
print (result)

from compas.rpc import Proxy
proxy = Proxy('compas.geometry')
result = proxy.add_vectors([2, -1, 3], [1, 0, 1])
print (result)