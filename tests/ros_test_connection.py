from compas_fab.backends import RosClient
with RosClient("192.168.43.28") as client:
    print('Connected:', client.is_connected)
