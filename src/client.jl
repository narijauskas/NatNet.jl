
mutable struct NatNetClient
    socket::UDPSocket
    group::IPAddr
    port::Integer
    host::IPAddr
    natnet_version::VersionNumber
end

function NatNetClient(;
        group::IPAddr = ip"239.255.42.99",
        port::Integer = 1511,
        host::IPAddr = ip"0.0.0.0",
        natnet_version = v"3.0.0",
    )
    socket = UDPSocket()
    bind(socket, host, port)
    join_multicast_group(socket, group)
    return NatNetClient(socket, group, port, host, natnet_version)
end

isopen(nn::NatNetClient) = isdefined(nn, :socket) && 3 == nn.socket.status

function open(nn::NatNetClient)
    if isopen(nn)
        @warn "UDPSocket is already open!"
    else
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
    else
        @warn "UDPSocket is already closed!"
    end
    return nn
end

function show(io::IO, nn::NatNetClient)
    print(io, "NatNetClient[$(nn.group):$(nn.port)] - ")
    print(io, crayon"bold", isopen(nn) ? crayon"green"("[open]") : crayon"red"("[closed]"))
end

# recv(nn::NatNetClient) = isopen(nn) && recv(nn.socket)
send(nn::NatNetClient, msg) = isopen(nn) && send(nn.socket, nn.group, nn.port, msg)

function recv(nn::NatNetClient)
    isopen(nn) || return
    pkt = Packet(recv(nn.socket))
    pkt(UInt16) == NAT_FRAMEOFDATA || return
    pkt(UInt16) # packet size
    pkt(NatNetFrame)
end
# #FUTURE:
# function recv(client::NatNetClient)
#     decode_natnet(recv(client.socket), client.natnet_version)
# end

# function decode_natnet(pkt, version)
#     try
#         _decode_natnet(pkt, version)
#     catch
#         @warn "failed to decode packet!"
#         pkt
#     end
# end


