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

#---------------------------------- unpacking basic types ----------------------------------#
# redefining uint8s as 16-64 or float32/64 
unpack(::Type{UInt16}, pkt::Vector{UInt8}, idx::Int) = ltoh(reinterpret(UInt16, pkt[idx:idx+1])[]), idx+2
unpack(::Type{UInt32}, pkt::Vector{UInt8}, idx::Int) = ltoh(reinterpret(UInt32, pkt[idx:idx+3])[]), idx+4
unpack(::Type{UInt64}, pkt::Vector{UInt8}, idx::Int) = ltoh(reinterpret(UInt64, pkt[idx:idx+7])[]), idx+8

unpack(::Type{Int16}, pkt::Vector{UInt8}, idx::Int) = ltoh(reinterpret(Int16, pkt[idx:idx+1])[]), idx+2
unpack(::Type{Int32}, pkt::Vector{UInt8}, idx::Int) = ltoh(reinterpret(Int32, pkt[idx:idx+3])[]), idx+4
unpack(::Type{Int64}, pkt::Vector{UInt8}, idx::Int) = ltoh(reinterpret(Int64, pkt[idx:idx+7])[]), idx+8

unpack(::Type{Float32}, pkt::Vector{UInt8}, idx::Int) = ltoh(reinterpret(Float32, pkt[idx:idx+3])[]), idx+4
unpack(::Type{Float64}, pkt::Vector{UInt8}, idx::Int) = ltoh(reinterpret(Float64, pkt[idx:idx+7])[]), idx+8

function unpack(::Type{Point}, pkt::Vector{UInt8}, idx::Int) #position
    x, idx = unpack(Float32, pkt, idx)
    y, idx = unpack(Float32, pkt, idx)
    z, idx = unpack(Float32, pkt, idx)

    return Point(x, y, z), idx
end

function unpack(::Type{Quaternion}, pkt::Vector{UInt8}, idx::Int) #orientation
    w,  idx = unpack(Float32, pkt, idx) 
    vx, idx = unpack(Float32, pkt, idx)
    vy, idx = unpack(Float32, pkt, idx)
    vz, idx = unpack(Float32, pkt, idx)

    return Quaternion(w, vx, vy, vz), idx
end

#---------------------------------- unpacking helper functions ----------------------------------#

# either the set of markers that make up the rigid body
# or the set of rigid bodies that make up the skeleton
function unpack_set(T::Type, pkt::Vector{UInt8}, idx::Int)
    n, idx = unpack(Int32, pkt, idx)
    set = Vector{T}(undef, n)
    for ix in 1:n
        set[ix], idx = unpack(T, pkt, idx)
    end
    return set, idx
end

#TODO: check the mocap output to see if the rigid body name appears
# name of the marker set
function unpack_name(pkt::Vector{UInt8}, idx::Int)
    jx = findnext(isequal(0x00), pkt, idx) # unpack until '\0' character
    name = String(pkt[idx:jx-1]) # jx-1 as to not include the '\0' separator in the name
    return name, jx+1
end

#---------------------------------- unpacking NatNet data types ----------------------------------#

# markers that are not associated with a rigid body
struct UnlabeledMarker
    pos::Point{3,Float32}
end

function unpack(::Type{UnlabeledMarker}, pkt::Vector{UInt8}, idx::Int)
    pos, idx = unpack(Point, pkt, idx)
    return UnlabeledMarker(pos), idx
end



struct MarkerSet
    name::String
    markers::Vector{UnlabeledMarker}
end

function unpack(::Type{MarkerSet}, pkt::Vector{UInt8}, idx::Int)
    name, idx    = unpack_name(pkt, idx)
    markers, idx = unpack_set(UnlabeledMarker, pkt, idx)
    return MarkerSet(name, markers), idx
end




struct RigidBody
    id::Int32
    pos::Point{3,Float32}
    rot::Quaternion{Float32}
    marker_err::Float32
    tracking_valid::Bool #if the rigid body can't be registered (a T or F)
end

function unpack(::Type{RigidBody}, pkt::Vector{UInt8}, idx::Int)
    id, idx         = unpack(Int32,      pkt, idx)
    pos, idx        = unpack(Point,      pkt, idx)
    rot, idx        = unpack(Quaternion, pkt, idx)
    marker_err, idx = unpack(Float32,    pkt, idx)
    param, idx      = unpack(Int16,      pkt, idx)
    tracking_valid  = (param & 0x01) != 0
    return RigidBody(id, pos, rot, marker_err, tracking_valid), idx
end




struct Skeleton
    id::Int32
    rigid_bodies::Vector{RigidBody}
end

function unpack(::Type{Skeleton}, pkt::Vector{UInt8}, idx::Int)
    id, idx           = unpack(Int32,         pkt, idx)
    rigid_bodies, idx = unpack_set(RigidBody, pkt, idx)
    return Skeleton(id, rigid_bodies)
end




struct LabeledMarker
    id::Int32
    pos::Point{3,Float32}
    size::Float32
    residual::Float32
    occluded::Bool
    point_cloud_solved::Bool
    model_solved::Bool
end

function unpack(::Type{LabeledMarker}, pkt::Vector{UInt8}, idx::Int)
    id, idx         = unpack(Int32,     pkt, idx)
    pos, idx        = unpack(Point,     pkt, idx)
    size, idx       = unpack(Float32,   pkt, idx)
    param, idx      = unpack(Int16,     pkt, idx)
    residual, idx   = unpack(Float32,   pkt, idx)
    occluded           = (param & 0x01) != 0 #TODO: check all 3 of these parameters in Motive
    point_cloud_solved = (param & 0x02) != 0 #TODO: look into the bound reconstruction through Motive
    model_solved       = (param & 0x04) != 0
    return LabeledMarker(id, pos, size, residual, occluded, point_cloud_solved, model_solved), idx
end


struct ChannelData #camera settings? or resolution? #TODO: check this 
    data::Vector{Float32}
end

function unpack(::Type{ChannelData}, pkt::Vector{UInt8}, idx::Int)
    data, idx = unpack_set(Float32, pkt, idx)
    return ChannelData(data), idx
end


struct DeviceData #for adding additional devices
    id::Int32
    channels::Vector{ChannelData}
end

function unpack(::Type{DeviceData}, pkt::Vector{UInt8}, idx::Int)
    id, idx       = unpack(Int32, pkt, idx)
    channels, idx = unpack(ChannelData, pkt, idx)
    return DeviceData(id, channels)
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

function unpack(::Type{Timestamp}, pkt::Vector{UInt8}, idx::Int)
    tc1, idx = unpack(UInt32, pkt, idx)
    tc2, idx = unpack(UInt32, pkt, idx)
    timecode = (tc1, tc2)
    timestamp, idx          = unpack(Float64, pkt, idx)
    exposure_timestamp, idx = unpack(Float64, pkt, idx)
    data_timestamp, idx     = unpack(Float64, pkt, idx)
    transmit_timestamp, idx = unpack(Float64, pkt, idx)
    return Timestamp(timecode, timestamp, exposure_timestamp, data_timestamp, transmit_timestamp), idx
end


struct NatNetFrame
    frame_number::Int
    markersets::Vector{MarkerSet}
    unlabeled_markers::Vector{UnlabeledMarker}
    rigid_bodies::Vector{RigidBody}
    skeletons::Vector{Skeleton}
    labeled_markers::Vector{LabeledMarker}
    force_plate_data::Vector{DeviceData}
    device_data::Vector{DeviceData}
    timestamp::Timestamp
    is_recording::Bool
    tracked_models_changed::Bool
end

# default "zero" constructor
function NatNetFrame()
    frame_number = 0
    markersets = MarkerSet[]
    unlabeled_markers = UnlabeledMarker[]
    rigid_bodies = RigidBody[]
    skeletons = Skeleton[]
    labeled_markers = LabeledMarker[]
    force_plate_data = DeviceData[]
    device_data = DeviceData[]
    timestamp = Timestamp()
    is_recording = false
    tracked_models_changed = false

    return NatNetFrame(frame_number, markersets, unlabeled_markers, rigid_bodies, 
        skeletons, labeled_markers, force_plate_data, device_data, timestamp,
        is_recording, tracked_models_changed
    )
end

function unpack(::Type{NatNetFrame}, pkt::Vector{UInt8}, idx::Int)
    frame_number, idx       = unpack(UInt32,                pkt, idx)
    markersets, idx         = unpack_set(MarkerSet,         pkt, idx)
    unlabeled_markers, idx  = unpack_set(UnlabeledMarker,   pkt, idx)
    rigid_bodies, idx       = unpack_set(RigidBody,         pkt, idx)
    skeletons, idx          = unpack_set(Skeleton,          pkt, idx)
    labeled_markers, idx    = unpack_set(LabeledMarker,     pkt, idx)
    force_plate_data, idx   = unpack_set(DeviceData,        pkt, idx)
    device_data, idx        = unpack_set(DeviceData,        pkt, idx)
    timestamp, idx          = unpack(Timestamp,             pkt, idx)
    param, idx              = unpack(UInt16,                pkt, idx)
    is_recording            = (param & 0x01) != 0
    tracked_models_changed  = (param & 0x01) != 0
    frame = NatNetFrame(frame_number, markersets, unlabeled_markers, rigid_bodies, skeletons, labeled_markers, force_plate_data, device_data, timestamp, is_recording, tracked_models_changed)
    return frame, idx
end

#TODO: comment this out once debugging is done?
function Base.show(io::IO, frame::NatNetFrame)
    compact = get(io, :compact, false)
    
    print("NatNetFrame")

    if !compact
        println(":")
        println("  frame: $(frame.frame_number)")
        println("  markersets: $(length(frame.markersets))")
        println("  unlabeled markers: $(length(frame.unlabeled_markers))")
        println("  rigid bodies: $(length(frame.rigid_bodies))")
        println("  skeletons: $(length(frame.skeletons))")
        println("  labeled markers: $(length(frame.labeled_markers))")
        println("  force plates: $(length(frame.force_plate_data))")
        println("  devices: $(length(frame.device_data))")
    end
end



function process_message(pkt::Vector{UInt8})
    idx = 1
    message_id, idx = unpack(UInt16, pkt, idx)
    packet_size, idx = unpack(UInt16, pkt, idx)

    if message_id == NAT_FRAMEOFDATA
        frame, idx = unpack(NatNetFrame, pkt, idx)
        return frame
    else
        @warn "unsupported message ID: $message_id"
    end
end