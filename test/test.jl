using NatNet

nn = NatNetClient()
open(nn)
recv(nn)



pkt = NatNet.Packet(recv(nn))

pkt(UInt16)
pkt(UInt16)
pkt(UInt32)
pkt(Vector{NatNet.MarkerSet})
pkt(Vector{NatNet.UnlabeledMarker})
pkt(Vector{NatNet.RigidBody})
pkt(Vector{NatNet.Skeleton})
pkt(Vector{NatNet.LabeledMarker})
pkt(Vector{NatNet.DeviceData})
pkt(Vector{NatNet.DeviceData})
pkt(NatNet.Timestamp)
pkt(UInt16)
pkt(NatNetFrame)