module NatNet

using Crayons
using Sockets
using Quaternions
using GeometryBasics: Point

import Sockets: recv, send
import Base: show
import Base: isopen, open, close
import Base: popfirst!

include("client.jl")
export NatNetClient
export recv, send

include("packets.jl")
export Point, QuaternionF32
export NatNetFrame
export UnlabeledMarker, LabeledMarker, MarkerSet
export RigidBody, Skeleton, DeviceData

end # module NatNet