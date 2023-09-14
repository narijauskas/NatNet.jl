using Base: @kwdef

@kwdef mutable struct NatNetClient
    group::IPAddr = ip"239.255.42.99"
    port::Integer = 1511
    host::IPAddr = ip"0.0.0.0"
    natnet_v::VersionNumber = v"3.0.0"
    socket::UDPSocket = UDPSocket()
end

isopen(nn::NatNetClient) = isdefined(nn, :socket) && 3 == nn.socket.status

function open(f::Function, nn::NatNetClient)
    open(nn)
    try
        f(nn)
    finally
        close(nn)
    end
end

function open(nn::NatNetClient)
    if !isopen(nn)
        nn.socket = UDPSocket()
        bind(nn.socket, nn.host, nn.port; reuseaddr = true)
        join_multicast_group(nn.socket, nn.group)
    end
    return nn
end

function close(nn::NatNetClient)
    if isopen(nn)
        leave_multicast_group(nn.socket, nn.group)
        close(nn.socket)
    end
    return nn
end

function show(io::IO, nn::NatNetClient)
    print(io, "NatNetClient[$(nn.group):$(nn.port)] - ")
    print(io, crayon"bold", isopen(nn) ? crayon"green"("[open]") : crayon"red"("[closed]"))
end

send(nn::NatNetClient, msg) = isopen(nn) && send(nn.socket, nn.group, nn.port, msg)

# recv(nn::NatNetClient) = isopen(nn) && recv(nn.socket)
function recv(nn::NatNetClient)
    isopen(nn) || return
    unpack = Unpacker(recv(nn.socket))
    unpack(UInt16) == NAT_FRAMEOFDATA || return
    unpack(UInt16) # packet size
    return unpack(NatNetFrame)
end
