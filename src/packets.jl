#NOTE: only supports NatNet v3.0 and later (because I was lazy)
# based on NatNetClient.py from the NatNet SDK v3.1, downloaded 7/26/2020
# https://optitrack.com/downloads/developer-tools.html#natnet-sdk

# Client/server message ids
const NAT_PING                  = 0 
const NAT_PINGRESPONSE          = 1
const NAT_REQUEST               = 2
const NAT_RESPONSE              = 3
const NAT_REQUEST_MODELDEF      = 4
const NAT_MODELDEF              = 5
const NAT_REQUEST_FRAMEOFDATA   = 6
const NAT_FRAMEOFDATA           = 7
const NAT_MESSAGESTRING         = 8
const NAT_DISCONNECT            = 9 
const NAT_UNRECOGNIZED_REQUEST  = 100

#---------------------------------- parsing helpers ----------------------------------#
mutable struct Unpacker
    bytes::Vector{UInt8}
end

decode(::Type{T}, bytes) where {T} = ltoh(reinterpret(T, bytes)[])::T
popfirst!(unpack::Unpacker, n::Integer) = map(x->popfirst!(unpack.bytes), 1:n)
popuntil!(f::Function, unpack::Unpacker) = popfirst!(unpack, findnext(f, unpack.bytes))[1:end-1]

(unpack::Unpacker)(::Type{T}) where {T}          = T((unpack(F) for F in fieldtypes(T))...)
(unpack::Unpacker)(::Type{T}) where {T<:Number}  = decode(T, popfirst!(unpack, sizeof(T)))
(unpack::Unpacker)(::Type{T}) where {T<:Tuple}   = Tuple(unpack(F) for F in fieldtypes(T))
(unpack::Unpacker)(::Type{Vector{T}}) where {T}  = [unpack(T) for _ in 1:unpack(Int32)]
(unpack::Unpacker)(::Type{<:AbstractString})     = String(popuntil!(isequal(0x00), unpack)) # unpack until '\0' character

# (pkt::Packet)(::Type{Point}) = Point((pkt(Float32) for _ in 1:3)...)
# (pkt::Packet)(::Type{Quaternion}) = Quaternion((pkt(Float32) for _ in 1:4)...)

#---------------------------------- NatNet data types ----------------------------------#

# markers that are not associated with a rigid body
struct UnlabeledMarker
    pos::Point{3,Float32}
end

struct MarkerSet
    name::String
    markers::Vector{UnlabeledMarker}
end

struct RigidBody
    id::Int32
    pos::Point{3,Float32}
    rot::Quaternion{Float32}
    marker_err::Float32
    param::UInt16
end

#if the rigid body can't be registered (a T or F)
tracking_valid(rb::RigidBody) = (rb.param & 0x01) != 0

struct Skeleton
    id::Int32
    rigid_bodies::Vector{RigidBody}
end

struct LabeledMarker
    id::Int32
    pos::Point{3,Float32}
    size::Float32
    param::UInt16
    residual::Float32
end

occluded(mkr)           = (mkr.param & 0x01) != 0 #TODO: check all 3 of these parameters in Motive
point_cloud_solved(mkr) = (mkr.param & 0x02) != 0 #TODO: look into the bound reconstruction through Motive
model_solved(mkr)       = (mkr.param & 0x04) != 0

struct ChannelData #camera settings? or resolution? #TODO: check this 
    data::Vector{Float32}
end

struct DeviceData #for adding additional devices
    id::Int32
    channels::Vector{ChannelData}
end

struct Timestamp
    timecode::Tuple{UInt32,UInt32}
    timestamp::Float64
    exposure_timestamp::Float64 # mid camera exposure timestamp
    data_timestamp::Float64 # camera data received timestamp
    transmit_timestamp::Float64 # packet transmit timestamp
end

# default "zero" constructor
Timestamp() = Timestamp((0,0), 0, 0, 0, 0)


struct NatNetFrame
    frame_number::UInt32
    markersets::Vector{MarkerSet}
    unlabeled_markers::Vector{UnlabeledMarker}
    rigid_bodies::Vector{RigidBody}
    skeletons::Vector{Skeleton}
    labeled_markers::Vector{LabeledMarker}
    force_plate_data::Vector{DeviceData}
    device_data::Vector{DeviceData}
    timestamp::Timestamp
    param::UInt16
end

# default "zero" constructor
function NatNetFrame()
    NatNetFrame(0,
        MarkerSet[],
        UnlabeledMarker[],
        RigidBody[],
        Skeleton[],
        LabeledMarker[],
        DeviceData[],
        DeviceData[],
        Timestamp(),
        0)
end

is_recording(x)            = (x.param & 0x01) != 0
tracked_models_changed(x)  = (x.param & 0x02) != 0


show(io::IO, frame::NatNetFrame) = print(io, "NatNetFrame[$(frame.frame_number)]")
function show(io::IO, ::MIME"text/plain", frame::NatNetFrame)
    println(io, frame, ":")
    if !get(io, :compact, false)
        println(io, "  $(length(frame.markersets)) ",        "markersets")
        println(io, "  $(length(frame.unlabeled_markers)) ", "unlabeled markers")
        println(io, "  $(length(frame.rigid_bodies)) ",      "rigid bodies")
        println(io, "  $(length(frame.skeletons)) ",         "skeletons")
        println(io, "  $(length(frame.labeled_markers)) ",   "labeled markers")
        println(io, "  $(length(frame.force_plate_data)) ",  "force plates")
        println(io, "  $(length(frame.device_data)) ",       "devices")
    end
end











# redefining uint8s as 16-64 or float32/64 
# unpack(::Type{UInt16}, pkt::Vector{UInt8}, idx::Int) = ltoh(reinterpret(UInt16, pkt[idx:idx+1])[]), idx+2
# unpack(::Type{UInt32}, pkt::Vector{UInt8}, idx::Int) = ltoh(reinterpret(UInt32, pkt[idx:idx+3])[]), idx+4
# unpack(::Type{UInt64}, pkt::Vector{UInt8}, idx::Int) = ltoh(reinterpret(UInt64, pkt[idx:idx+7])[]), idx+8

# unpack(::Type{Int16}, pkt::Vector{UInt8}, idx::Int) = ltoh(reinterpret(Int16, pkt[idx:idx+1])[]), idx+2
# unpack(::Type{Int32}, pkt::Vector{UInt8}, idx::Int) = ltoh(reinterpret(Int32, pkt[idx:idx+3])[]), idx+4
# unpack(::Type{Int64}, pkt::Vector{UInt8}, idx::Int) = ltoh(reinterpret(Int64, pkt[idx:idx+7])[]), idx+8

# unpack(::Type{Float32}, pkt::Vector{UInt8}, idx::Int) = ltoh(reinterpret(Float32, pkt[idx:idx+3])[]), idx+4
# unpack(::Type{Float64}, pkt::Vector{UInt8}, idx::Int) = ltoh(reinterpret(Float64, pkt[idx:idx+7])[]), idx+8

# function unpack(::Type{Point}, pkt::Vector{UInt8}, idx::Int) #position
#     x, idx = unpack(Float32, pkt, idx)
#     y, idx = unpack(Float32, pkt, idx)
#     z, idx = unpack(Float32, pkt, idx)

#     return Point(x, y, z), idx
# end

# function unpack(::Type{Quaternion}, pkt::Vector{UInt8}, idx::Int) #orientation
#     w,  idx = unpack(Float32, pkt, idx) 
#     vx, idx = unpack(Float32, pkt, idx)
#     vy, idx = unpack(Float32, pkt, idx)
#     vz, idx = unpack(Float32, pkt, idx)

#     return Quaternion(w, vx, vy, vz), idx
# end

#---------------------------------- unpacking helper functions ----------------------------------#

# # either the set of markers that make up the rigid body
# # or the set of rigid bodies that make up the skeleton
# function unpack_set(T::Type, pkt::Vector{UInt8}, idx::Int)
#     n, idx = unpack(Int32, pkt, idx)
#     set = Vector{T}(undef, n)
#     for ix in 1:n
#         set[ix], idx = unpack(T, pkt, idx)
#     end
#     return set, idx
# end

# #TODO: check the mocap output to see if the rigid body name appears
# # name of the marker set
# function unpack_name(pkt::Vector{UInt8}, idx::Int)
#     jx = findnext(isequal(0x00), pkt, idx) # unpack until '\0' character
#     name = String(pkt[idx:jx-1]) # jx-1 as to not include the '\0' separator in the name
#     return name, jx+1
# end



# function unpack(::Type{UnlabeledMarker}, pkt::Vector{UInt8}, idx::Int)
#     pos, idx = unpack(Point, pkt, idx)
#     return UnlabeledMarker(pos), idx
# end


# function unpack(::Type{MarkerSet}, pkt::Vector{UInt8}, idx::Int)
#     name, idx    = unpack_name(pkt, idx)
#     markers, idx = unpack_set(UnlabeledMarker, pkt, idx)
#     return MarkerSet(name, markers), idx
# end



# function unpack(::Type{RigidBody}, pkt::Vector{UInt8}, idx::Int)
#     id, idx         = unpack(Int32,      pkt, idx)
#     pos, idx        = unpack(Point,      pkt, idx)
#     rot, idx        = unpack(Quaternion, pkt, idx)
#     marker_err, idx = unpack(Float32,    pkt, idx)
#     param, idx      = unpack(Int16,      pkt, idx)
#     tracking_valid  = (param & 0x01) != 0
#     return RigidBody(id, pos, rot, marker_err, tracking_valid), idx
# end


# # pkt(Int16)


# function unpack(::Type{Skeleton}, pkt::Vector{UInt8}, idx::Int)
#     id, idx           = unpack(Int32,         pkt, idx)
#     rigid_bodies, idx = unpack_set(RigidBody, pkt, idx)
#     return Skeleton(id, rigid_bodies)
# end





# function unpack(::Type{LabeledMarker}, pkt::Vector{UInt8}, idx::Int)
#     id, idx         = unpack(Int32,     pkt, idx)
#     pos, idx        = unpack(Point,     pkt, idx)
#     size, idx       = unpack(Float32,   pkt, idx)
#     param, idx      = unpack(Int16,     pkt, idx)
#     residual, idx   = unpack(Float32,   pkt, idx)
#     occluded           = (param & 0x01) != 0 #TODO: check all 3 of these parameters in Motive
#     point_cloud_solved = (param & 0x02) != 0 #TODO: look into the bound reconstruction through Motive
#     model_solved       = (param & 0x04) != 0
#     return LabeledMarker(id, pos, size, residual, occluded, point_cloud_solved, model_solved), idx
# end



# function unpack(::Type{ChannelData}, pkt::Vector{UInt8}, idx::Int)
#     data, idx = unpack_set(Float32, pkt, idx)
#     return ChannelData(data), idx
# end

# function unpack(::Type{DeviceData}, pkt::Vector{UInt8}, idx::Int)
#     id, idx       = unpack(Int32, pkt, idx)
#     channels, idx = unpack(ChannelData, pkt, idx)
#     return DeviceData(id, channels)
# end

# function unpack(::Type{Timestamp}, pkt::Vector{UInt8}, idx::Int)
#     tc1, idx = unpack(UInt32, pkt, idx)
#     tc2, idx = unpack(UInt32, pkt, idx)
#     timecode = (tc1, tc2)
#     timestamp, idx          = unpack(Float64, pkt, idx)
#     exposure_timestamp, idx = unpack(Float64, pkt, idx)
#     data_timestamp, idx     = unpack(Float64, pkt, idx)
#     transmit_timestamp, idx = unpack(Float64, pkt, idx)
#     return Timestamp(timecode, timestamp, exposure_timestamp, data_timestamp, transmit_timestamp), idx
# end


# function unpack(::Type{NatNetFrame}, pkt::Vector{UInt8}, idx::Int)
#     frame_number, idx       = unpack(UInt32,                pkt, idx)
#     markersets, idx         = unpack_set(MarkerSet,         pkt, idx)
#     unlabeled_markers, idx  = unpack_set(UnlabeledMarker,   pkt, idx)
#     rigid_bodies, idx       = unpack_set(RigidBody,         pkt, idx)
#     skeletons, idx          = unpack_set(Skeleton,          pkt, idx)
#     labeled_markers, idx    = unpack_set(LabeledMarker,     pkt, idx)
#     force_plate_data, idx   = unpack_set(DeviceData,        pkt, idx)
#     device_data, idx        = unpack_set(DeviceData,        pkt, idx)
#     timestamp, idx          = unpack(Timestamp,             pkt, idx)
#     param, idx              = unpack(UInt16,                pkt, idx)
#     is_recording            = (param & 0x01) != 0
#     tracked_models_changed  = (param & 0x02) != 0
#     frame = NatNetFrame(frame_number, markersets, unlabeled_markers, rigid_bodies, skeletons, labeled_markers, force_plate_data, device_data, timestamp, is_recording, tracked_models_changed)
#     return frame, idx
# end




# function process_message(pkt::Vector{UInt8})
#     idx = 1
#     message_id, idx = unpack(UInt16, pkt, idx)
#     packet_size, idx = unpack(UInt16, pkt, idx)

#     if message_id == NAT_FRAMEOFDATA
#         frame, idx = unpack(NatNetFrame, pkt, idx)
#         return frame
#     else
#         @warn "unsupported message ID: $message_id"
#     end
# end



#---------------------------------- unpacking helper functions ----------------------------------#



# (pkt::Packet)(::Type{UnlabeledMarker}) = UnlabeledMarker(pkt(Point))
# (pkt::Packet)(::Type{MarkerSet}) = MarkerSet(pkt(Name), pkt(Vector{UnlabeledMarker}))
# (pkt::Packet)(::Type{RigidBody}) = RigidBody(pkt(Int32), pkt(Point), pkt(Quaternion), pkt(Float32), pkt(Int16))
# (pkt::Packet)(::Type{Skeleton}) = Skeleton(pkt(Int32), pkt(Vector{RigidBody}))

# (pkt::Packet)(::Type{Timestamp}) = Timestamp(pkt(UInt32, 2), pkt(Float64, 4)...)


# (pkt::Packet)(::Type{Tuple{T...}}) where {T...} = Tuple(pkt(T) for T in (T...))
