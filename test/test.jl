using NatNet

client = NatNetClient()
open(client)
frame = recv(client)
close(client)

frame = open(recv, client)