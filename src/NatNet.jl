module NatNet

using Crayons
using Sockets
using Quaternions
using GeometryBasics: Point
import Sockets: recv, send
import Base: show
import Base: isopen, open, close
# using Base.Threads: @spawn

include("client.jl")
export NatNetClient, recv, send

include("packets.jl")
export process_message
export NatNetFrame

end
